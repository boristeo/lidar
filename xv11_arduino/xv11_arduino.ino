/* 
  Code to interface with an XV_11 lidar
  Author: Boris Teodorovch
*/

namespace {
	// Configuration
	const uint16_t SAMPLES_CT = 60;
	const bool USE_CHECKSUM = false;

	namespace xv11 {
		struct xv11_packet {
			struct sample {
				uint16_t dist : 14; // the distances are in millimeters and have a 14 bit resolution
				bool bad      : 1;  // mask 0x8000 - bad certainty
				bool failed   : 1;  // mask 0x4000 - failed measurement
				uint16_t q;					// quality of the sample as reported by lidar
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
		
		void read_packet(xv11_packet *p) {
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
					in_packet_bytes_read = Serial2.readBytes(&p->start, 1);
					if (in_packet_bytes_read < 1 || p->start != 0xfa) { break; }
					state = index;
					
				case index:
					in_packet_bytes_read += Serial2.readBytes(&p->index, 1);
					if (in_packet_bytes_read < 2) { break; }

					if (p->index >= 0xa0 && p->index <= 0xf9) {
						state = data;
					}
					else if (p->index == 0xfa) { // Turns out the previous byte was not a start byte
						in_packet_bytes_read--; // drop total bytes read back down so that after re-reading in_packet_bytes_read == 2
						break;
					}
					else {
						state = idle; // abort
						break;
					}
					
				case data:
					in_packet_bytes_read += Serial2.readBytes(((uint8_t *) p) + in_packet_bytes_read, sizeof(*p) - in_packet_bytes_read);
					if (in_packet_bytes_read < sizeof(p)) { break; }
				
					return;
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



	template<size_t N>
	class car {
		double absolute_angle = 0;
		uint16_t lidar_distances[N];

		bool relative_position(size_t relative_angle, double *angle, uint16_t *distance) const {
			return false;	
		}
		double relative_angle(double absolute_angle) const {
			return (absolute_angle - relative_angle) % (2 * M_PI);
		}

		void update_position() {
			
		}
		
	public:
		void on_lidar_rot() {
			update_position();
		}

		void process_lidar_packet(const xv11::xv11_packet &p) {
			for (auto i = 0; i < sizeof(p.samples) / sizeof(p.samples[0]); i++) {
				
				lidar_distances[p.start_angle() + i] = p.samples[i].dist
									| p.samples[i].bad << 14 
									| p.samples[i].failed << 15;

			}
		}

		
	};

}

void setup(){
  Serial.begin(250000);
  Serial2.begin(115200);
}


void loop() {
  static xv11::xv11_packet p;
  static unsigned int in_packet_start_index = 0;
  static unsigned int last_in_packet_start_index = 0;
  static unsigned int out_packet_start_index = 0;
  static size_t full_rots_offset = 0;

  static out_packet<SAMPLES_CT> head;
	static car<360> c;
  
	for (;;) {
		xv11::read_packet(&p);
		if (!p.verify_checksum() && USE_CHECKSUM) { continue; }
		// Data valid; use
		
		in_packet_start_index = p.start_angle();

		if (last_in_packet_start_index > in_packet_start_index && SAMPLES_CT >= 360) {
			full_rots_offset += 360; // New packets will be indexed from 0, create artificial offset
			c.on_lidar_rot();
		}
			
		size_t data_index = full_rots_offset + in_packet_start_index - out_packet_start_index; 
		if (data_index >= SAMPLES_CT) {
			// new in_packet won't fit in out_packet, send it and prepare for next
			head.start_angle = out_packet_start_index;
			head.rpm = p.rpm;
			Serial.write((uint8_t *) &head, sizeof(head));
			full_rots_offset = 0;
			out_packet_start_index = in_packet_start_index;
		}
		for (auto i = 0; i < 4; i++) {
			memcpy(head.samples + data_index + i, p.samples + i, 2);
		}
		c.process_lidar_packet(p);
		
		last_in_packet_start_index = in_packet_start_index;
	}

}
