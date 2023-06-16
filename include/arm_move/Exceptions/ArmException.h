#include <iostream>

class ArmException: public std::exception
{
private:
    std::string msg;
public:
    ArmException(std::string _msg): msg(_msg) {} 
    
    std::string what()
    {
        return msg;
    }
};
