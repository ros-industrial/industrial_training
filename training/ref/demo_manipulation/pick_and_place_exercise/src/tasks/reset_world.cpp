/*
 * open_gripper.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <pick_and_place_exercise/pick_and_place.h>

/*    SET OBJECT IN WORLD
  Goal:
    - Attaches or detaches target to arms end-effector link.
    - Publishes object marker for visualization.
  Hints:
*/
void PickAndPlace::reset_world()
{

	// removing attached objects from robot
	set_attached_object(false);

	// get new sensor snapshot
	detect_box_pick();

}


