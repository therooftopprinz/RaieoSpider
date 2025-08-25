#include <cmath>
#include <vector>
#include <tuple>

#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>

#include "schedule_context.hpp"

template<typename S, typename... T>
void log(S& serial, const char* msg, T... t)
{
    char m[1024];
    snprintf(m, sizeof(m), msg, t...);
    serial.println(m);
}

#define PCA9685_ADDRESS 0x60
#define OE_PIN 34
#define I2C_SDA 21
#define I2C_SCL 22

const uint MIN_PULSE = 500;
const uint MAX_PULSE = 2500;

Adafruit_PWMServoDriver pwm(PCA9685_ADDRESS);

BluetoothSerial SerialBT;
const char* btDeviceName = "RaieoSpider";

template<typename... T>
void log(const char* msg, T... t)
{
    char m[1024];
    snprintf(m, sizeof(m), msg, t...);
    Serial.println(m);
    SerialBT.println(m);
}

struct servo_config_t
{
    unsigned quadrant;
    unsigned pos;
    unsigned mid;
    float div;
};

constexpr auto N_SERVO = 16;

servo_config_t servo_configs[N_SERVO] =
{
/**  0 **/    {1, 0, 1430,    10.78},
/**  1 **/    {1, 1, 1490,   -10.56},
/**  2 **/    {1, 2, 1430,    10.56},
/**  3 **/    {1, 3, 0,      -16.67},
/**  4 **/    {4, 0, 1455,    11.94},
/**  5 **/    {4, 1, 1570,   -10.44},
/**  6 **/    {4, 2, 1500,    11.11},
/**  7 **/    {4, 3, 0,      -16.67},
/**  8 **/    {3, 0, 1560,   -11.89},
/**  9 **/    {3, 1, 1550,    10.89},
/** 10 **/    {3, 2, 1580,   -10.11},
/** 11 **/    {3, 3, 0,      -16.67},
/** 12 **/    {2, 0, 1470,   -11.56},
/** 13 **/    {2, 1, 1560,    11.56},
/** 14 **/    {2, 2, 1470,   -11.33},
/** 15 **/    {2, 3, 0,      -16.67}
};

class servo
{
public:
    servo() = default;

    servo(unsigned channel, unsigned mid, float div)
        : channel(channel)
        , mid(mid)
        , div(div)
    {}

    void set_angle(float a)
    {
        set_raw(mid + a*div);
    }

    void set_raw(unsigned raw)
    {
        pwm.writeMicroseconds(channel, raw);
    }

private:
    unsigned channel;
    unsigned mid;
    float div;
};

servo servos[N_SERVO];

struct leg_config_t
{
    float origin_x;
    float origin_y;
    float origin_z;
    float A;
    float B;
    float D; // negative on left
    float alpha_offset;
    float beta_offset;
    float gamma_offset;
    float alpha_hlim;
    float alpha_llim;
    float beta_hlim;
    float beta_llim;
    float gamma_hlim;
    float gamma_llim;
};

constexpr auto N_LEG = 4;

leg_config_t leg_configs[4] =
{
//                x     y    z     A    B    D    a   b    g   ah  al   bh  bl  gh   gl
/** Q1 **/    { 150,  150, -125, 150, 100,  100, 90, 90,  45, 180, 30, 180, 45, 90, -45},
/** Q2 **/    {-150,  150, -125, 150, 100, -100, 90, 90,  45, 180, 30, 180, 45, 90, -45},
/** Q3 **/    {-150, -150, -125, 150, 100, -100, 90, 90, -45, 180, 30, 180, 45, 45, -90},
/** Q4 **/    { 150, -150, -125, 150, 100,  100, 90, 90, -45, 180, 30, 180, 45, 45, -90}
};

float sqr(float x)
{
    return x*x;
}

float sgn(float x)
{
    return x>=0 ? 1 : -1;
}

float deg(float x)
{
    return 180*x/PI;
}

class leg
{
public:
    leg() = default;
    leg(unsigned quadrant,
        float origin_x, float origin_y, float origin_z,
        float A, float B, float D,
        float alpha_offset, float beta_offset, float gamma_offset,
        float alpha_hlim, float alpha_llim,
        float beta_hlim, float beta_llim,
        float gamma_hlim, float gamma_llim)
        : quadrant(quadrant)
        , origin_x(origin_x)
        , origin_y(origin_y)
        , origin_z(origin_z)
        , A(A)
        , AA(A*A)
        , B(B)
        , BB(B*B)
        , AB(A*B)
        , D(D)
        , alpha_offset(alpha_offset)
        , beta_offset(beta_offset)
        , gamma_offset(gamma_offset)
        , alpha_hlim(alpha_hlim)
        , alpha_llim(alpha_llim)
        , beta_hlim(beta_hlim)
        , beta_llim(beta_llim)
        , gamma_hlim(gamma_hlim)
        , gamma_llim(gamma_llim)
    {}

    void assign_servo(unsigned pos, servo* s)
    {
        if      (0 == pos) root = s;
        else if (1 == pos) mid  = s;
        else if (2 == pos) claw = s;
    }

    void center(float x, float y, float z)
    {
        log(Serial, "CENTER_UPDATE %d %4.1f %4.1f %4.1f", quadrant, x,y,z);
        off_x = x;
        off_y = y;
        off_z = z;

        position(cx, cy, cz);
    }

    void position(float x, float y, float z)
    {
        log(Serial, "LEG_UPDATE %d %4.1f %4.1f %4.1f", quadrant, x,y,z);

        cx = x;
        cy = y;
        cz = z;

        x += origin_x;
        y += origin_y;
        z += origin_z;

        x += off_x;
        y += off_y;
        z += off_z;

        // log(Serial, "leg[Q%d] | R(%.3f,%.3f,%.3f)", quadrant, x,y,z);

        float g = std::atan(y/std::fabs(x));
        float R = std::sqrt(x*x+y*y);

        // log(Serial, "leg[Q%d] | g_deg=%f R=%f", quadrant, deg(g), R);

        float  X = sgn(D)*R;
        float& Y = z;

        float CC = sqr(X-D)+sqr(Y);
        float C  = std::sqrt(CC);
        float BC = B*C;

        // log(Serial, "leg[Q%d] | A=%f AA=%f B=%f BB=%f AB=%f C=%f CC=%f BC=%f D=%f", quadrant, A, AA, B, BB, AB, C, CC, BC, D);

        float b1 = std::acos((BB+CC-AA)/(2*BC));
        float b2 = -sgn(Y) * std::asin((-sgn(-D)*X + sgn(-D)*D)/C) + (Y>=0 ? PI : 0);
        // log(Serial, "leg[Q%d] | b1_deg=%f", quadrant, deg(b1));
        // log(Serial, "leg[Q%d] | b2_deg=%f", quadrant, deg(b2));

        float a = std::acos((AA+BB-CC)/(2*AB));
        float a_deg = deg(a);
        float b_deg = deg(b1+b2);
        float g_deg = deg(g);

        // log(Serial, "leg[Q%d] | a_deg=%f", quadrant, a_deg);

        float a_adj = a_deg - alpha_offset;
        float b_adj = b_deg - beta_offset;
        float g_adj = g_deg - gamma_offset;

        // log(Serial, "leg[Q%d] | (%.3f,%.3f,%.3f) -> a=%.3f b=%.3f g=%.3f", quadrant, x,y,z, a_adj, b_adj, g_adj);

        bool a_oor = a_deg > alpha_hlim || a_deg < alpha_llim;
        bool b_oor = b_deg > beta_hlim  || b_deg < beta_llim;
        bool g_oor = g_deg > gamma_hlim || g_deg < gamma_llim;

        if (a_oor || b_oor || g_oor)
        {
            log(Serial, "leg[Q%d] | OOR(%d,%d,%d)", quadrant, a_oor, b_oor, g_oor);
            return;
        }

        if (std::isnan(a_deg) || std::isnan(b_deg) || std::isnan(g_deg))
        {
            log(Serial, "leg[Q%d] | OOR NAN", quadrant);
            return;
        }

        root->set_angle(g_adj);
        mid ->set_angle(b_adj);
        claw->set_angle(a_adj);
    }

    std::tuple<float,float,float,float,float,float> get_cxyz()
    {
        return std::make_tuple(off_x,off_y,off_z,cx,cy,cz);
    }

private:
    unsigned quadrant;

    float origin_x;
    float origin_y;
    float origin_z;
    float A;
    float AA;
    float B;
    float BB;
    float AB;
    float D;
    float alpha_offset;
    float beta_offset;
    float gamma_offset;
    float alpha_hlim;
    float alpha_llim;
    float beta_hlim;
    float beta_llim;
    float gamma_hlim;
    float gamma_llim;

    float cx;
    float cy;
    float cz;
    float off_x = 0;
    float off_y = 0;
    float off_z = 0;

    servo* root = nullptr;
    servo* mid  = nullptr;
    servo* claw = nullptr;
};

leg legs[N_LEG];

class pose_storage
{
public:
    void add_leg(leg* l)
    {
        legs.emplace_back(l);
    }

    void save(unsigned pose_id, unsigned pose_index)
    {
        if (pose_id >= poses.size())
        {
            poses.resize(pose_id + 1);
        }

        auto& pose = poses[pose_id];
        
        if (pose_index >= pose.size())
        {
            pose.resize(pose_index + 1);
        }

        auto& poselegs = pose[pose_index];
        poselegs.clear();

        for (auto& leg : legs)
        {
            poselegs.emplace_back(leg->get_cxyz());
        }
    }

    void save(unsigned pose_id, unsigned pose_index, unsigned leg_index, float ox, float oy, float oz, float x, float y, float z)
    {
        if (pose_id >= poses.size())
        {
            poses.resize(pose_id + 1);
        }

        auto& pose = poses.at(pose_id);

        if (pose_index >= pose.size())
        {
            pose.resize(pose_index + 1);
        }

        auto& poselegs = pose.at(pose_index);

        if (leg_index >= poselegs.size())
        {
            poselegs.resize(leg_index + 1);
        }

        poselegs.at(leg_index) = std::make_tuple(ox, oy, oz, x, y, z);
    }

    void load(unsigned pose_id, unsigned pose_index, bool inc_idx = false)
    {
        current_pose_id  = pose_id;
        current_pose_idx = pose_index;

        if (inc_idx)
        {
            current_pose_idx++;
        }

        if (pose_id >= poses.size())
        {
            current_pose_id = 0;
            log(Serial, "poses[%d][%d] | Invalid pose_id", pose_id, pose_index);
            return;
        }
        
        auto& pose = poses.at(pose_id);
        
        if (pose_index >= pose.size())
        {
            current_pose_idx = 0;
            log(Serial, "poses[%d][%d] | Invalid pose_index", pose_id, pose_index);
            return;
        }

        auto& poselegs = pose.at(pose_index);

        unsigned i = 0;
        for (auto& leg : legs)
        {
            // auto [x,y,z] = poselegs[i++];
            auto& pl = poselegs[i++];
            auto ox = std::get<0>(pl);
            auto oy = std::get<1>(pl);
            auto oz = std::get<2>(pl);
            auto x  = std::get<3>(pl);
            auto y  = std::get<4>(pl);
            auto z  = std::get<5>(pl);
            leg->center(ox, oy, oz);
            leg->position(x,y,z);
        }
    }

    void tick()
    {
        log(Serial, "poses.tick() | id=%d idx=%d", current_pose_id, current_pose_idx);
        load(current_pose_id, current_pose_idx, true);
    }

    void dump()
    {
        unsigned i = 0;
        for (auto& pose : poses)
        {
            unsigned j = 0;
            for (auto& plegs : pose)
            {
                unsigned k = 0;
                for (auto& leg : plegs)
                {
                    auto ox = std::get<0>(leg);
                    auto oy = std::get<1>(leg);
                    auto oz = std::get<2>(leg);
                    auto x  = std::get<3>(leg);
                    auto y  = std::get<4>(leg);
                    auto z  = std::get<5>(leg);
                    log("POSE %u %u %u %4.0f %4.0f %4.0f %4.0f %4.0f %4.0f", i, j, k, ox, oy, oz, x, y, z);
                    k++;
                }
                j++;
            }
            i++;
        }
    }

private:
    std::vector<leg*> legs;
    using legs_pose_t = std::vector<std::tuple<float,float,float,float,float,float>>;
    std::vector<std::vector<legs_pose_t>> poses;
    unsigned current_pose_id = 0;
    unsigned current_pose_idx = 0;
};

pose_storage poses;

void reset_position()
{
    for (auto i=0; i<N_SERVO; i++)
    {
        servos[i].set_angle(0);
    }
}

void setup()
{
    Serial.begin(115200);
    SerialBT.begin(btDeviceName);

    log(Serial, "Bluetooth started! Device name: %s\n", btDeviceName);
    log(Serial, "Pair with this device in your phone's Bluetooth settings");

    pinMode(OE_PIN, OUTPUT);
    digitalWrite(OE_PIN, LOW);

    Wire.begin(I2C_SDA, I2C_SCL);
    if (pwm.begin())
    {
        pwm.setPWMFreq(50);
        log(Serial, "PCA9685 initialized");
    }
    else
    {
        log(Serial, "PCA9685 not found!");
        while (1)
            delay(100);
    }

    for (auto i=0u; i<N_SERVO; i++)
    {
        servos[i] = servo(i, servo_configs[i].mid, servo_configs[i].div);
    }

    for (auto i=0u; i<N_LEG; i++)
    {
        auto& c = leg_configs[i];
        legs[i] = leg(i+1,c.origin_x, c.origin_y, c.origin_z, c.A, c.B, c.D, c.alpha_offset, c.beta_offset, c.gamma_offset, c.alpha_hlim, c.alpha_llim, c.beta_hlim, c.beta_llim, c.gamma_hlim, c.gamma_llim);
        poses.add_leg(&legs[i]);
    }

    for (auto i=0u; i<N_SERVO; i++)
    {
        auto& c = servo_configs[i];
        legs[c.quadrant-1].assign_servo(c.pos, servos+i);
    }

    delay(1000);
    reset_position();

    log(Serial, "Ready");
}

template <typename S>
void readInput(S& serial)
{
    while (serial.available() > 0)
    {
        char cmd = serial.read();

        if ('A' == cmd)
        {
            unsigned channel = serial.parseInt();
            float    angle   = serial.parseFloat();
            servos[channel].set_angle(angle);
            log(serial, "servos[%d].angle=%f", channel, angle);
        }
        else if ('R' == cmd)
        {
            unsigned channel = serial.parseInt();
            unsigned raw     = serial.parseInt();
            servos[channel].set_raw(raw);
            log(serial, "servos[%d].raw=%d", channel, raw);
        }
        else  if ('C' == cmd)
        {
            float x = serial.parseFloat();
            float y = serial.parseFloat();
            float z = serial.parseFloat();
            for (auto i=0u; i<N_LEG; i++)
            {
                legs[i].center(x,y,z);
            }
            log(serial, "legs.center(%.1f,%.1f,%.1f)", x,y,z);
        }
        else  if ('L' == cmd)
        {
            unsigned quadrant = serial.parseInt();
            float x = serial.parseFloat();
            float y = serial.parseFloat();
            float z = serial.parseFloat();
            legs[quadrant-1].position(x,y,z);
            log(serial, "leg[Q%d].position=(%.1f,%.1f,%.1f)", quadrant, x,y,z);
        }
        else if ('X' == cmd)
        {
            reset_position();
            log(serial, "servos.reset_position()");
        }
        else if ('S' == cmd)
        {
            unsigned pose_id  = serial.parseInt();
            unsigned pose_idx = serial.parseInt();
            log(serial, "poses[%d][%d].save()", pose_id, pose_idx);
            poses.save(pose_id, pose_idx);
        }
        else if ('E' == cmd)
        {
            unsigned pose_id  = serial.parseInt();
            unsigned pose_idx = serial.parseInt();
            unsigned pose_leg = serial.parseInt();
            float ox = serial.parseFloat();
            float oy = serial.parseFloat();
            float oz = serial.parseFloat();
            float  x = serial.parseFloat();
            float  y = serial.parseFloat();
            float  z = serial.parseFloat();
            log(serial, "poses[%d][%d].save(%d, %.1f,%.1f,%.1f, %.1f,%.1f,%.1f)", pose_id, pose_idx, pose_leg, ox, oy, oz, x, y, z);
            poses.save(pose_id, pose_idx, pose_leg, ox, oy, oz, x, y, z);
        }
        else  if ('D' == cmd)
        {
            unsigned pose_id  = serial.parseInt();
            unsigned pose_idx = serial.parseInt();
            log(serial, "poses[%d][%d].load()", pose_id, pose_idx);
            poses.load(pose_id, pose_idx);
        }
        else  if ('P' == cmd)
        {
            log(serial, "poses.dump()");
            poses.dump();
        }
        else  if ('T' == cmd)
        {
            log(serial, "poses.tick()");
            poses.tick();
        }

        serial.readStringUntil('\n');
    }
}

void loop()
{
    readInput(SerialBT);
    readInput(Serial);
}
