/* 
  Code to interface with an XV_11 lidar
  Author: Boris Teodorovch
*/

// Configuration
const uint16_t SAMPLES_CT = 60;
const bool USE_CHECKSUM = false;

uint16_t checksum(void *buf, size_t size) {
  uint32_t chk32 = 0;
  for (size_t i = 0; i < size / 2; i++) {
    chk32 = (chk32 << 1) + ((uint16_t *) buf)[i];
  }
  return ((chk32 & 0x7fff) + (chk32 >> 15)) & 0x7fff;
}
 
void setup(){
  Serial.begin(250000);
  Serial2.begin(115200);
}

struct in_packet {
  uint8_t start; // should be 0xfa
  uint8_t index; // 4 x 1 degree samples per packet:  0 <= (index - 0xa0) < (360 / 4) 
  uint16_t rpm;  // real rpm is actually rpm / 64

  uint16_t s0_d: 14; // the distances are in millimeters and have a 14 bit resolution
  bool s0_bad: 1; // mask 0x8000 - bad certainty
  bool s0_failed: 1; // mask 0x4000 - failed measurement
  uint16_t s0_q;
  uint16_t s1_d: 14;
  bool s1_bad: 1;
  bool s1_failed: 1;
  uint16_t s1_q;
  uint16_t s2_d: 14;
  bool s2_bad: 1;
  bool s2_failed: 1;
  uint16_t s2_q;
  uint16_t s3_d: 14;
  bool s3_bad: 1;
  bool s3_failed: 1;
  uint16_t s3_q;

  uint16_t checksum;
};

template <unsigned int N>
struct out_packet {
  char sync[4] = {'\n', 'a', 'b', '\n'};
  uint16_t start_angle;
  uint16_t rpm;
  uint16_t count = N * 2;
  uint16_t samples[N]; 
};


enum State {
  idle,
  index,
  data
};

void loop() {
  static in_packet p;
  static size_t in_packet_bytes_read = 0;
  static unsigned int in_packet_start_index = 0;

  static out_packet<SAMPLES_CT> head;
  static unsigned int out_packet_start_index = 0;
  static size_t full_rots_offset = 0;
  

  static State state = 0;
  switch(state) {
  case idle:
    in_packet_bytes_read = Serial2.readBytes(&p.start, 1);
    if (in_packet_bytes_read < 1 || p.start != 0xfa) { break; }
    state = index;
    
  case index:
    in_packet_bytes_read += Serial2.readBytes(&p.index, 1);
    if (in_packet_bytes_read < 2) { break; }

    if (p.index >= 0xa0 && p.index <= 0xf9) {
      auto last_chunk = in_packet_start_index;
      in_packet_start_index = (p.index - 0xa0) * 4; // 0 <= index < 359

      if (last_chunk > in_packet_start_index && SAMPLES_CT >= 360) {
        full_rots_offset += 360; // New packets will be indexed from 0, create artificial offset
      }
      state = data;
    }
    else if (p.index == 0xfa) { // Turns out the previous byte was not a start byte
      in_packet_bytes_read--; // drop total bytes read back down so that after re-reading in_packet_bytes_read == 2
      break;
    }
    else {
      state = idle; // abort
      break;
    }
    
  case data:
    in_packet_bytes_read += Serial2.readBytes(((uint8_t *) &p) + in_packet_bytes_read, sizeof(p) - in_packet_bytes_read);
    if (in_packet_bytes_read < sizeof(p)) { break; }
  
    if (checksum(&p, sizeof(p) - sizeof(p.checksum)) == p.checksum || !USE_CHECKSUM) {
      // Data valid; use
      size_t data_index = full_rots_offset + in_packet_start_index - out_packet_start_index; 
      if (data_index >= SAMPLES_CT) {
        // new in_packet won't fit in out_packet, send it and prepare for next
        head.start_angle = out_packet_start_index;
        head.rpm = p.rpm;
        Serial.write((uint8_t *) &head, sizeof(head));
        full_rots_offset = 0;
        out_packet_start_index = in_packet_start_index;
      }
      head.samples[data_index + 0] = p.s0_d | p.s0_bad << 14 | p.s0_failed << 15;
      head.samples[data_index + 1] = p.s1_d | p.s1_bad << 14 | p.s1_failed << 15;
      head.samples[data_index + 2] = p.s2_d | p.s2_bad << 14 | p.s2_failed << 15;
      head.samples[data_index + 3] = p.s3_d | p.s3_bad << 14 | p.s3_failed << 15;
    }
    state = idle;
    break;
    
  }
}
