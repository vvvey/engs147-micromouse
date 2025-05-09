
class IR {
    public:
        IR(int id, int type);
        IR(int id, int type, int pin);
        void begin(int id, int pin);
        float getDistance(int id);
    
    private:
        int ir_id;
        int ir_type;
        int ir_pin;
        int ir_analog_value;
        int ir_distance;
        float v2d_4_30(float v);
        float v2d_2_15(float v);
};
