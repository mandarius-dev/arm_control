#include <arm_move/Source/Manipulator.h>

Manipulator::Manipulator(ros::NodeHandle *nh)
{

    pl = new Planner(nh);
    tg = new TrajectoryGeneration(nh);
    ph = new PathExecutor(nh);
    wo = new Workarea(nh);
}

/**
 * @brief Starts the interpreter and then executes the given commands
 * 
 */
void Manipulator::arm_initialize()
{
    pl->read_command_syntax();
    if(pl->read_command())
    {
        if(tg->generate_instruction_buffer(pl->get_plan_buffer()))
        {
            wo->read_forbidden_zones();
            wo->set_buffers(pl->get_plan_buffer(), tg->get_instruction_buffer(), tg->get_full_path_points());

            if(wo->check_workarea())
            {
                tg->print_full_path_points();
                tg->print_instruction_buffer();

                ph->execute_instructions(tg->get_instruction_buffer(), tg->get_full_path_points(), pl->get_loop_index());
            }
        }
    }
}