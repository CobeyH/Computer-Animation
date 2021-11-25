////////////////////////////////////////////////////
// // Template code for  CS 174C
////////////////////////////////////////////////////

#ifdef WIN32
#include <windows.h>
#endif


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <shared/defs.h>

#include "shared/opengl.h"

#include <string.h>
#include <util/util.h>
#include <GLModel/GLModel.h>
#include "anim.h"
#include "animTcl.h"
#include "myScene.h"
#include "BoidSystem.h"
#include "BoidSimulator.h"
#include "FoodSystem.h"


int g_testVariable = 10;

SETVAR myScriptVariables[] = {
	"testVariable", TCL_LINK_INT, (char *) &g_testVariable,
	"",0,(char *) NULL
};


//---------------------------------------------------------------------------------
//			Hooks that are called at appropriate places within anim.cpp
//---------------------------------------------------------------------------------

// start or end interaction
void myMouse(int button, int state, int x, int y)
{

	// let the global resource manager know about the new state of the mouse 
	// button
	GlobalResourceManager::use()->setMouseButtonInfo( button, state );

	if( button == GLUT_LEFT_BUTTON && state == GLUT_DOWN )
	{
		double transformedX = x / (2000.0 / 12.0) - 6;
		double transformedY = y / (-1000.0 / 12.0) + 6;
		BaseSystem* systemGetter = GlobalResourceManager::use()->getSystem("boidSystem");
		BoidState state;
		systemGetter->getState((double*) &state);
		double closest = 10000;
		int idOfClosest = -1;
		for (auto it = state.flocks->begin(); it != state.flocks->end(); ++it) {
			for (std::list<Boid*>::iterator bIt = it->members.begin(); bIt != it->members.end(); ++bIt) {
				Boid* nextBoid = *bIt;
				double distanceToNextBoid = sqrt(pow(nextBoid->position[0] - transformedX, 2) + pow(nextBoid->position[1] - transformedY, 2));
				if (distanceToNextBoid < closest) {
					closest = distanceToNextBoid;
					idOfClosest = nextBoid->id;
				}
			}
		}
		animTcl::OutputMessage("Closest Boid ID %d", idOfClosest);
	}
	if( button == GLUT_LEFT_BUTTON && state == GLUT_UP )
	{

	}
}	// myMouse

// interaction (mouse motion)
void myMotion(int x, int y)
{

	GLMouseButtonInfo updatedMouseButtonInfo = 
		GlobalResourceManager::use()->getMouseButtonInfo();

	if( updatedMouseButtonInfo.button == GLUT_LEFT_BUTTON )
	{
		animTcl::OutputMessage(
			"My mouse motion callback received a mousemotion event\n") ;
	}

}	// myMotion


void MakeScene(void)
{

	/* 
	
	This is where you instantiate all objects, systems, and simulators and 
	register them with the global resource manager

	*/

	/* SAMPLE SCENE */

	bool success;

	// register a system
	BoidSystem* bSystem = new BoidSystem( "boidSystem");

	success = GlobalResourceManager::use()->addSystem( bSystem, true );

	// make sure it was registered successfully
	assert( success );

	FoodSystem* fSystem = new FoodSystem("foodSystem");

	success = GlobalResourceManager::use()->addSystem(fSystem, true);

	BoidSimulator* boidSim = new BoidSimulator("boidSim", bSystem);

	boidSim->registerFoodSystem(fSystem);

	success = GlobalResourceManager::use()->addSimulator(boidSim, true);

	// make sure it was registered successfully
	assert(success);

	/* END SAMPLE SCENE */


}	// MakeScene

// OpenGL initialization
void myOpenGLInit(void)
{
	animTcl::OutputMessage("Initialization routine was called.");

}	// myOpenGLInit

void myIdleCB(void)
{
	
	return;

}	// myIdleCB

void myKey(unsigned char key, int x, int y)
{
	 animTcl::OutputMessage("My key callback received a key press event\n");
	return;

}	// myKey

static int testGlobalCommand(ClientData clientData, Tcl_Interp *interp, int argc, myCONST_SPEC char **argv)
{
	 animTcl::OutputMessage("This is a test command!");
    animTcl::OutputResult("100") ;
	return TCL_OK;

}	// testGlobalCommand

void mySetScriptCommands(Tcl_Interp *interp)
{

	// here you can register additional generic (they do not belong to any object) 
	// commands with the shell

	Tcl_CreateCommand(interp, "test", testGlobalCommand, (ClientData) NULL,
					  (Tcl_CmdDeleteProc *)	NULL);

}	// mySetScriptCommands
