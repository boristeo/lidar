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
      //return distances[(size_t) (resolution() * angle / (2 * M_PI))] - 1;
      return distances[179] > 140 ? distances[179] : 0xffff;
    }

    virtual bool read_full_rot() {
      static size_t c = 0;
      static xv11_packet p;

      if (serial->available()) {
        c += read_packet(&p);
        for (auto i = 0; i < 4; i++) {
          distances[p.start_index() + i] = p.samples[i].dist;
                    //| p.samples[i].bad << 14 
                    //| p.samples[i].failed << 15;
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
          in_packet_bytes_read += serial->readBytes(((uint8_t *) packet) + in_packet_bytes_read, sizeof(*packet) - in_packet_bytes_read);
          if (in_packet_bytes_read < sizeof(*packet)) { break; }
        
          return 4;
        }
      }
    }
    
  };
   


  class car {
  public:
    class Command {
      protected:
        car* my_car;
      public:
        virtual void init() = 0;
        virtual void on_tick() = 0;
        virtual bool is_done() const = 0;
        virtual void finish() = 0;

    };

    enum motor { fl, fr, rl, rr };
    enum mmode { fwd, rev, brake, release };
    enum special { 
      turn180 = 129,
      door_tg = 130,
      door_op = 131,
      door_cl = 132
    };

    double heading = 0;
    const unsigned long long MILLIS_PER_TICK = 250;

    HardwareSerial *drivetrain;
    Lidar *lidar;


    Command **commands;
    size_t command_count = 0;
    size_t current_command = 0;

    
    car (HardwareSerial *s, Lidar *l): drivetrain(s), lidar(l) {}

    void on_lidar_rot() {
      update_position();
    }
    void on_tick() {
      static bool initialized = false;

      if (!initialized) {
        commands[current_command]->init();
        initialized = true;
      }
      if (commands[current_command]->is_done()) {
        commands[current_command]->finish();
        current_command = (current_command + 1) % command_count;
        initialized = false;
      }
      else {
        commands[current_command]->on_tick();
      }
    }

    void load_commands(Command **cs, size_t count) {
      commands = cs;
      command_count = count;
    }

    void drive() {
      volatile long last_millis = millis();
      for (;;) {
        if (lidar->read_full_rot()) {
          on_lidar_rot();
        }
        if (millis() - last_millis >= MILLIS_PER_TICK) {
          on_tick();
          last_millis = millis();
        }
      }
    }

    void dt_write(motor m, mmode mode, uint8_t v) {
      drivetrain->write(m | mode << 2 | v << 4);
    }
    void dt_write(special s) {
      drivetrain->write(s);
    }
    void update_position() {
      
    }

    void dt_write_all(mmode mode, uint8_t v) {
      dt_write(fr, mode, v);
      dt_write(fl, mode, v);
      dt_write(rr, mode, v);
      dt_write(rl, mode, v);
    }
  };

  class DriveForMillis: public car::Command {
  private:
    unsigned long long target_time;
    unsigned long long time_passed;
    car::mmode mode;
    uint8_t spd;

  public:
    DriveForMillis(car* my_car, unsigned long long target_time, car::mmode mode, uint8_t spd) {
      this->target_time = target_time;
      this->my_car = my_car;
      this->mode = mode;
      this->spd = spd;
    }

    virtual void init() {
      time_passed = 0;
    }
    virtual void on_tick() {
      my_car->dt_write_all(mode, spd);
      
      time_passed += my_car->MILLIS_PER_TICK;
    }

    virtual bool is_done() const {
      return time_passed >= target_time;
    }

    virtual void finish() {
      my_car->dt_write_all(car::mmode::brake, 0);
    }
  };

  class Turn180_brya: public car::Command {
  unsigned long long time_passed = 0;
  public:
    Turn180_brya(car *my_car) {
      this->my_car = my_car;
    }

    virtual void init() {
      time_passed = 0;
      my_car->dt_write(car::special::turn180);
    }
    virtual void on_tick() {
      my_car->dt_write_all(car::mmode::brake, 0);
      time_passed += my_car->MILLIS_PER_TICK;
    }

    virtual bool is_done() const {
      return time_passed >= 2000;
    }

    virtual void finish() {
    }
  };

  class Turn180_ket: public car::Command {
  private:
    static const unsigned long long TIME_TO_TURN = 500;
    static const uint8_t SPEED = 7;
    unsigned long long time_passed;

  public:
    Turn180_ket(car* my_car) {
      this->my_car = my_car;
    }

    virtual void init() {
      time_passed = 0;
    }
    virtual void on_tick() {
      my_car->dt_write(car::motor::fl, car::mmode::fwd, SPEED);
      my_car->dt_write(car::motor::fr, car::mmode::rev, SPEED);
      my_car->dt_write(car::motor::rl, car::mmode::fwd, SPEED);
      my_car->dt_write(car::motor::rr, car::mmode::rev, SPEED);

      time_passed += my_car->MILLIS_PER_TICK;
    }

    virtual bool is_done() const {
      return time_passed >= TIME_TO_TURN;
    }

    virtual void finish() {
      my_car->dt_write_all(car::mmode::brake, 0);
    }
  };

  class DriveUntilObstacle: public car::Command {
  private:
    uint16_t distance_threshold;
    uint8_t spd;

  public:
    DriveUntilObstacle(car* my_car, uint16_t distance_threshold, uint8_t spd) {
     this->my_car = my_car;
     this->distance_threshold = distance_threshold;
     this->spd = spd;
    }

    virtual void init() {
    }

    virtual void on_tick() {
      my_car->dt_write_all(car::mmode::fwd, spd);
    }

    virtual bool is_done() const {
      auto d = my_car->lidar->distance_at(0);
      Serial.println(d);
      return d <= distance_threshold;
    }

    virtual void finish() {
      my_car->dt_write_all(car::mmode::brake, 0);
    }
  };


}



void setup(){
  Serial.begin(250000);
  Serial1.begin(250000);
  Serial2.begin(115200);
  
  static XV11 lidar(&Serial2);
  static car c(&Serial1, &lidar);
  car::Command *cs[] = { 
    new DriveUntilObstacle(&c, 300, 3),
    new Turn180_brya(&c)
  };
  c.load_commands(cs, sizeof(cs) / sizeof(*cs));

  c.drive();
}


void loop() {}
