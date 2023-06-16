#include <iostream>

/**
 * Structure class used to store the information of the read command 
 * 
 */
class Command
{
private:
    std::string name;
    std::string first_argument;
    std::string second_argument;
public:
    Command(std::string _name, std::string _first_argument, std::string _second_argument): 
        name(_name), 
        first_argument(_first_argument),
        second_argument(_second_argument) 
    {}

    std::string get_name();
    std::string get_first_argument();
    std::string get_second_argument();
};

std::string Command::get_name()
{
    return name;
}

std::string Command::get_first_argument()
{
    return first_argument;
}

std::string Command::get_second_argument()
{
    return second_argument;
}

#pragma once