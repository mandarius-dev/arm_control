#include <iostream>

/**
 * Structure class that stores the arm command
 * 
 */
class InstructionCommand
{
private:
    std::string name;
    int from;
    int to;

    bool joint_space;

    int arg;
public:
    InstructionCommand(std::string _name, int _from, int _to, bool _joint_space);
    InstructionCommand(std::string _name, int _arg);

    std::string get_name();
    int get_from();
    int get_to();

    bool is_joint_space();

    int get_arg();
};

InstructionCommand::InstructionCommand(std::string _name, int _from, int _to, bool _joint_space): name(_name), from(_from), to(_to), joint_space(_joint_space)
{
}

InstructionCommand::InstructionCommand(std::string _name, int _arg): name(_name), arg(_arg)
{
}

std::string InstructionCommand::get_name()
{
    return name;
}

int InstructionCommand::get_from()
{
    return from;
}

int InstructionCommand::get_to()
{
    return to;
}

int InstructionCommand::get_arg()
{
    return arg;
}

bool InstructionCommand::is_joint_space()
{
    return joint_space;
}

#pragma once