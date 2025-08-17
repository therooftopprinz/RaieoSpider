#ifndef __SCHEDULE_CONTEXT_HPP__
#define __SCHEDULE_CONTEXT_HPP__

struct schedule_context
{
    unsigned long last = 0;
    unsigned long interval = 100;
    unsigned long diff = 0;
    operator bool()
    {
        unsigned long now = millis();
        if (now - last >= interval)
        {
            diff = now - last;
            last = now;
            return true;
        }
        return false;
    }
};

#endif // __SCHEDULE_CONTEXT_HPP__