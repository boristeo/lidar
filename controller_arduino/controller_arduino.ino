/* 
  Code to interface with an XV_11 lidar
  Author: Boris Teodorovch
*/
#define SERIAL_BUFFER_SIZE 256

namespace {
  class Lidar {
  public:
    virtual bool read_full_rot() = 0;

    // getters
    virtual uint16_t distance_at(double angle) const = 0;
    virtual size_t resolution() const = 0;
  };


  class XV11: public Lidar {
  private:
    struct xv11_packet {
      struct sample {
        uint16_t dist : 14; // the distances are in millimeters and have a 14 bit resolution
        bool bad      : 1;  // mask 0x8000 - bad certainty
        bool failed   : 1;  // mask 0x4000 - failed measurement
        uint16_t q;         // quality of the sample as reported by lidar
      };

      uint8_t start; // should be 0xfa
      uint8_t index; // 4 x 1 degree samples per packet:  0 <= (index - 0xa0) < (360 / 4) 
      uint16_t rpm;  // real rpm is actually rpm / 64
      sample samples[4];
      uint16_t checksum;
      
      bool verify_checksum() const {
        uint32_t chk32 = 0;
        for (size_t i = 0; i < sizeof(*this) - sizeof(this->checksum) / 2; i++) {
          chk32 = (chk32 << 1) + ((uint16_t *) this)[i];
        }
        return ((chk32 & 0x7fff) + (chk32 >> 15)) & 0x7fff;
      }
      uint16_t start_index() const {
        return (index - 0xa0) << 2;
      }
    };

    static const size_t SAMPLES_PER_ROT = 360;
    uint16_t distances[SAMPLES_PER_ROT];
    HardwareSerial *serial;

  public:
    XV11(HardwareSerial *s): serial(s) {}

    virtual uint16_t distance_at(double angle) const {
      return distances[(size_t) (resolution() * angle / (2 * M_PI))];
    }

    virtual bool read_full_rot() {
      static size_t c = 0;
      static xv11_packet p;

      if (serial->available()) {
        c += read_packet(&p);
        for (auto i = 0; i < sizeof(p.samples) / sizeof(p.samples[0]); i++) {
          distances[p.start_index() + i] = p.samples[i].dist
                    | p.samples[i].bad << 14 
                    | p.samples[i].failed << 15;
        }
      } 
      if (c > SAMPLES_PER_ROT) {
        c = 0;
        return true;
      }
      else {
        return false;
      }
    }

    virtual size_t resolution() const {
      return SAMPLES_PER_ROT;
    }

  private:
    size_t read_packet(xv11_packet *packet) {
      size_t in_packet_bytes_read = 0;

      enum State {
        idle,
        index,
        data
      };

      auto state = idle;
      for (;;) {
        switch(state) {
        case idle:
          in_packet_bytes_read = serial->readBytes(&packet->start, 1);
          if (in_packet_bytes_read < 1 || packet->start != 0xfa) { break; }
          state = index;
          
        case index:
          in_packet_bytes_read += serial->readBytes(&packet->index, 1);
          if (in_packet_bytes_read < 2) { break; }

          if (packet->index >= 0xa0 && packet->index <= 0xf9) {
            state = data;
          }
          else if (packet->index == 0xfa) { // Turns out the previous byte was not a start byte
            in_packet_bytes_read--; // drop total bytes read back down so that after re-reading in_packet_bytes_read == 2
            break;
          }
          else {
            state = idle; // abort
            break;
          }
          
        case data:
          in_packet_bytes_read += serial->readBytes(((uint8_t *) &packet) + in_packet_bytes_read, sizeof(*packet) - in_packet_bytes_read);
          if (in_packet_bytes_read < sizeof(*packet)) { break; }
        
          return 4;
        }
      }
    }
    
  };
   

  class car {
  private:
    size_t ticks = 0;
    double heading = 0;

    HardwareSerial *drivetrain;
    Lidar *lidar;

    enum motor { fl, fr, rl, rr };
    enum mmode { fwd, rev, brake, release };
    enum special { 
      turn180 = 129,
      door_tg = 130,
      door_op = 131,
      door_cl = 132
    };
    
  public:
    car (HardwareSerial *s, Lidar *l): drivetrain(s), lidar(l) {}

    void on_lidar_rot() {
      update_position();
    }
    void on_tick() {
      ticks++;
    //  dt_write(fr, rev, 3);
    }
    void drive() {
      volatile auto last_millis = millis();
      for (;;) {
        if (lidar->read_full_rot()) {
          on_lidar_rot();
        }
        if (millis() - last_millis >= 250) {
          on_tick();
          last_millis = millis();
        }
      }
    }

  private:
    void dt_write(motor m, mmode mode, uint8_t v) {
      drivetrain->write(m | mode << 2 | v << 4);
    }
    void dt_write(special s) {
      drivetrain->write(s);
    }
    void update_position() {
      
    }
  };
}

void setup(){
  Serial.begin(250000);
  Serial1.begin(250000);
  Serial2.begin(115200);
  
  static XV11 lidar(&Serial2);
  static car c(&Serial1, &lidar);
  c.drive();
}


void loop() {

  /*
  static unsigned int in_packet_start_index = 0;
  static unsigned int last_in_packet_start_index = 0;
  static unsigned int out_packet_start_index = 0;
  static size_t full_rots_offset = 0;

  static out_packet<SAMPLES_CT> head;
  
  for (;;) {
    if (!lidar.read_packet()) { continue; }
    
    in_packet_start_index = lidar.packet.start_angle();

    if (last_in_packet_start_index > in_packet_start_index && SAMPLES_CT >= 360) {
      full_rots_offset += 360; // New packets will be indexed from 0, create artificial offset
      c.on_lidar_rot();
    }
      
    size_t data_index = full_rots_offset + in_packet_start_index - out_packet_start_index; 
    if (data_index >= SAMPLES_CT) {
      // new in_packet won't fit in out_packet, send it and prepare for next
      head.start_angle = out_packet_start_index;
      head.rpm = lidar.packet.rpm;
      Serial.write((uint8_t *) &head, sizeof(head));
      full_rots_offset = 0;
      out_packet_start_index = in_packet_start_index;
    }
    for (auto i = 0; i < 4; i++) {
      memcpy(head.samples + data_index + i, lidar.packet.samples + i, 2);
    }
    c.process_lidar_packet(lidar.packet);
    
    last_in_packet_start_index = in_packet_start_index;
  }
    */

}
