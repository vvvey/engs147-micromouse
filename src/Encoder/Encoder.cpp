#include <AxisEncoderShield3.h>    
class Encoder {
    public:
        Encoder(int id, float ticks_per_rev);
    
        void begin();
        void update(unsigned long current_time_ms);  // pass millis()
    
        float getVelocity();    // returns omega (rad/s)
        long getPosition();     // encoder ticks
        void reset();
    
    private:
        int encoder_id;
        float ticks_per_rev;
        long prev_ticks;
        unsigned long prev_time_ms;
        float omega_rad_s;
};
