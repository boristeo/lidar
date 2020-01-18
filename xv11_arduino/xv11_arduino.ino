/* 
  Code to interface with an XV_11 lidar
  Author: Boris Teodorovch
*/

namespace {
  // Configuration
  const uint16_t SAMPLES_CT = 60;
  const bool USE_CHECKSUM = false;

  class xv11 {
  public:
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
      uint16_t start_angle() const {
        return (index - 0xa0) << 2;
      }
    };

    static const size_t SAMPLES_PER_ROT = 360;
    xv11_packet packet;
    
  private:
    HardwareSerial *serial;

  public:
    xv11(HardwareSerial *s): serial(s) {}
    
    bool read_packet(bool verify_checksum = true) {
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
          in_packet_bytes_read = serial->readBytes(&packet.start, 1);
          if (in_packet_bytes_read < 1 || packet.start != 0xfa) { break; }
          state = index;
          
        case index:
          in_packet_bytes_read += serial->readBytes(&packet.index, 1);
          if (in_packet_bytes_read < 2) { break; }

          if (packet.index >= 0xa0 && packet.index <= 0xf9) {
            state = data;
          }
          else if (packet.index == 0xfa) { // Turns out the previous byte was not a start byte
            in_packet_bytes_read--; // drop total bytes read back down so that after re-reading in_packet_bytes_read == 2
            break;
          }
          else {
            state = idle; // abort
            break;
          }
          
        case data:
          in_packet_bytes_read += serial->readBytes(((uint8_t *) &packet) + in_packet_bytes_read, sizeof(packet) - in_packet_bytes_read);
          if (in_packet_bytes_read < sizeof(packet)) { break; }
        
          return !verify_checksum || packet.verify_checksum();
        }
      }
    }
  };
   

  template <unsigned int N>
  struct out_packet {
    char sync[4] = {'\n', 'a', 'b', '\n'};
    uint16_t start_angle;
    uint16_t rpm;
    uint16_t count = N * 2;
    uint16_t samples[N]; 
  };



  class car {
    size_t ticks = 0;
    double heading = 0;
    uint16_t lidar_distances[xv11::SAMPLES_PER_ROT];

    HardwareSerial *drivetrain;
    xv11 *lidar;

/*
    bool relative_position(size_t relative_angle, double *angle, uint16_t *distance) const {
      return false; 
    }
    size_t discretize_angle(double angle) const {
      return ((size_t) (xv11::SAMPLES_PER_ROT * round(angle) / (2 * M_PI))) % xv11::SAMPLES_PER_ROT;
    }
    double relative_angle(double absolute_angle) const {
      return (absolute_angle - heading) % (2 * M_PI);
    }

*/
    void update_position() {
      
    }

    enum motor { fl, fr, rl, rr };
    enum mmode { fwd, rev, brake, release };
    enum special { 
      turn180 = 129,
      door_tg = 130,
      door_op = 131,
      door_cl = 132
    };
    void dt_write(motor m, mmode mode, uint8_t v) {
      drivetrain->write(m | mode << 2 | v << 4);
    }
    void dt_write(special s) {
      drivetrain->write(s);
    }
    
  public:
    car (HardwareSerial *s, xv11 *l): drivetrain(s), lidar(l) {}

    void on_lidar_rot() {
      update_position();
      ticks += 1;
    }

    void process_lidar_packet(const xv11::xv11_packet &p) {
      for (auto i = 0; i < sizeof(p.samples) / sizeof(p.samples[0]); i++) {
        
        lidar_distances[p.start_angle() + i] = p.samples[i].dist
                  | p.samples[i].bad << 14 
                  | p.samples[i].failed << 15;

      }
    }

    void drive() {
      auto last_angle = lidar->packet.start_angle();
      for (;;) {
        if (!lidar->read_packet()) { continue; }
        if (lidar->packet.start_angle() < last_angle) { on_lidar_rot(); }
        last_angle = lidar->packet.start_angle();
        process_lidar_packet(lidar->packet);
//        if (ticks < 40) {
//          dt_write(fr, fwd, 3);
//        }
//        else {
//          dt_write(fr, rev, 3);
//          if (ticks > 80) {
//            ticks = 0;
//          }
//        }
      }
    }
  };

}

void setup(){
  Serial.begin(250000);
  Serial1.begin(250000);
  Serial2.begin(115200);
  
  static xv11 lidar(&Serial2);
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
