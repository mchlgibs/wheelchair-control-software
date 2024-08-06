//Required include files
#include <stdio.h>	
#include <string>
#include <iostream>
#define _USE_MATH_DEFINES // For M_PI
#include <cmath>          // For trigonometric functions and M_PI
#include <limits>         // For numeric_limits
#include <stdexcept>      // For invalid_argument
#include <corecrt_math_defines.h>
#include "pubSysCls.h"	

using namespace sFnd;

// Send message and wait for newline
void msgUser(const char *msg) {
	std::cout << msg;
	(void)getchar();
}

// Constants for the distances from fulcrum to pivot
const double A = 20.5; // inches
const double B = 13.5; // inches

int getRollInput(const std::string& variableName) {
	int value;
	while (true) {
		std::cout << "Enter value for " << variableName << ": "; // << " (between -5 and 5): ";
		std::cin >> value;
			/*if (std::cin >> value) {
				if (value >= -5 && value <= 5) {
					return value; // Return the valid input
				}
				else {
					std::cout << "Invalid input. Please enter an integer between -10 and 10.\n";
				}
			}
			else {
				std::cin.clear(); // Clear error flag
				std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Ignore wrong input
				std::cout << "Invalid input. Please enter an integer.\n";
			}*/
			return value; // Return the valid input
	}
}
int getPitchInput(const std::string& variableName) {
	int value;
	while (true) {
		std::cout << "Enter value for " << variableName << ": "; // "(between -3 and 8): ";
		std::cin >> value;
		/*if (std::cin >> value) {
			if (value >= -3 && value <= 8) {
				return value; // Return the valid input
			}
			else {
				std::cout << "Invalid input. Please enter an integer between -6 and 11.\n";
			}
		}
		else {
			std::cin.clear(); // Clear error flag
			std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Ignore wrong input
			std::cout << "Invalid input. Please enter an integer.\n";
		}*/
		return value; // Return the valid input
	}
}

//*********************************************************************************
//This program will load configuration files onto each node connected to the port, then executes
//sequential repeated moves on each axis.
//*********************************************************************************

#define ACC_LIM_RPM_PER_SEC	100000
#define VEL_LIM_RPM			4000
#define MOVE_DISTANCE_CNTS	40000	
#define NUM_MOVES			5
#define TIME_TILL_TIMEOUT	50000	//The timeout used for homing(ms)

int main(int argc, char* argv[])
{
	msgUser("Motion Example starting. Press Enter to continue.");

	size_t portCount = 0;
	std::vector<std::string> comHubPorts;


	//Create the SysManager object. This object will coordinate actions among various ports
	// and within nodes. In this example we use this object to setup and open our port.
	SysManager* myMgr = SysManager::Instance();							//Create System Manager myMgr

	//This will try to open the port. If there is an error/exception during the port opening,
	//the code will jump to the catch loop where detailed information regarding the error will be displayed;
	//otherwise the catch loop is skipped over
	try
	{ 
		
		SysManager::FindComHubPorts(comHubPorts);
		printf("Found %zd SC Hubs\n", comHubPorts.size());

		for (portCount = 0; portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX; portCount++) {
			
			myMgr->ComHubPort(portCount, comHubPorts[portCount].c_str()); 	//define the first SC Hub port (port 0) to be associated 
											// with COM portnum (as seen in device manager)
		}

		if (portCount < 0) {
			
			printf("Unable to locate SC hub port\n");

			msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key

			return -1;  //This terminates the main program
		}
		//printf("\n I will now open port \t%i \n \n", portnum);
		myMgr->PortsOpen(portCount);				//Open the port

		for (size_t i = 0; i < portCount; i++) {
			IPort &myPort = myMgr->Ports(i);

			printf(" Port[%d]: state=%d, nodes=%d\n",
				myPort.NetNumber(), myPort.OpenState(), myPort.NodeCount());



			//Once the code gets past this point, it can be assumed that the Port has been opened without issue
			//Now we can get a reference to our port object which we will use to access the node objects

			for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
				// Create a shortcut reference for a node
				INode& theNode = myPort.Nodes(iNode);

				theNode.EnableReq(false);				//Ensure Node is disabled before loading config file

				myMgr->Delay(200);


				//theNode.Setup.ConfigLoad("Config File path");


				printf("   Node[%d]: type=%d\n", int(iNode), theNode.Info.NodeType());
				printf("            userID: %s\n", theNode.Info.UserID.Value());
				printf("        FW version: %s\n", theNode.Info.FirmwareVersion.Value());
				printf("          Serial #: %d\n", theNode.Info.SerialNumber.Value());
				printf("             Model: %s\n", theNode.Info.Model.Value());

				//The following statements will attempt to enable the node.  First,
				// any shutdowns or NodeStops are cleared, finally the node is enabled
				theNode.Status.AlertsClear();					//Clear Alerts on node 
				theNode.Motion.NodeStopClear();	//Clear Nodestops on Node  				
				theNode.EnableReq(true);					//Enable node 
				//At this point the node is enabled
				printf("Node \t%zi enabled\n", iNode);
				double timeout = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT;	//define a timeout in case the node is unable to enable
				//This will loop checking on the Real time values of the node's Ready status
				while (!theNode.Motion.IsReady()) {
					if (myMgr->TimeStampMsec() > timeout) {
						printf("Error: Timed out waiting for Node %zd to enable\n", iNode);
						msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
						return -2;
					}
				}
			}
			for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
				// Create a shortcut reference for a node
				INode& theNode = myPort.Nodes(iNode);
				//At this point the Node is enabled, and we will now check to see if the Node has been homed
				//Check the Node to see if it has already been homed, 
				if (theNode.Motion.Homing.HomingValid())
				{
					if (theNode.Motion.Homing.WasHomed())
					{
						printf("Node %zd has already been homed, current position is: \t%8.0f \n", iNode, theNode.Motion.PosnMeasured.Value());
						printf("Rehoming Node... \n");
					}
					else
					{
						printf("Node [%zd] has not been homed.  Homing Node now...\n", iNode);
					}
					//Now we will home the Node
					theNode.Motion.Homing.Initiate();
					
					printf("Node completed homing\n");
				}
				else {
					printf("Node[%zd] has not had homing setup through ClearView.  The node will not be homed.\n", iNode);
				}
			}
			// Monitor all nodes for completion of homing
			bool allNodesHomed;
			do {
				allNodesHomed = true;
				for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
					INode& theNode = myPort.Nodes(iNode);
					if (!theNode.Motion.Homing.WasHomed()) {
						allNodesHomed = false; // if any node is not yet homed, set to false
					}
				}
				if (!allNodesHomed) {
					myMgr->Delay(100); // delay a bit before checking again to reduce CPU usage
				}
			} while (!allNodesHomed);

			printf("All nodes successfully homed.\n");
			
			///////////////////////////////////////////////////////////////////////////////////////
			//At this point we will execute 10 rev moves sequentially on each axis
			//////////////////////////////////////////////////////////////////////////////////////
			try {
				int rollDegrees = getRollInput("roll");
				int pitchDegrees = getPitchInput("pitch");

				// Convert degrees to radians for trigonometric calculations
				double r = rollDegrees * M_PI / 180;
				double p = pitchDegrees * M_PI / 180;

				// Calculate h1 and h2 based on the given formulas
				double h1 = B * sin(r) + A * sin(p);
				double h2 = A * sin(p) - B * sin(r);

				// Calculate the distance in counts for the heights
				int move1 = int(h1 * -65000); // TODO: check integer conversion
				int move2 = int(h2 * -65000);

				std::cout << "You entered roll: " << rollDegrees << " degrees and pitch: " << pitchDegrees << " degrees." << std::endl;
				std::cout << "Calculated h1: " << h1 << " inches." << std::endl;
				std::cout << "Calculated h2: " << h2 << " inches." << std::endl;
				std::cout << "Calculated move1: " << move1 << " counts." << std::endl;
				std::cout << "Calculated move2: " << move2 << " counts." << std::endl;

				for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
					// Create a shortcut reference for a node
					INode& theNode = myPort.Nodes(iNode);

					theNode.Motion.MoveWentDone();						//Clear the rising edge Move done register

					theNode.AccUnit(INode::RPM_PER_SEC);				//Set the units for Acceleration to RPM/SEC
					theNode.VelUnit(INode::RPM);						//Set the units for Velocity to RPM
					theNode.Motion.AccLimit = ACC_LIM_RPM_PER_SEC;		//Set Acceleration Limit (RPM/Sec)
					theNode.Motion.VelLimit = VEL_LIM_RPM;				//Set Velocity Limit (RPM)
					if (iNode == 0) {
						printf("Moving Node \t%zi \n", iNode);
						theNode.Motion.MovePosnStart(move1);
						printf("%f estimated time.\n", theNode.Motion.MovePosnDurationMsec(move1));
						double timeout = myMgr->TimeStampMsec() + theNode.Motion.MovePosnDurationMsec(move1) + 100;			//define a timeout in case the node is unable to enable
					}

					else if (iNode == 1) {
						printf("Moving Node \t%zi \n", iNode);
						theNode.Motion.MovePosnStart(move2);
						printf("%f estimated time.\n", theNode.Motion.MovePosnDurationMsec(move2));
						double timeout = myMgr->TimeStampMsec() + theNode.Motion.MovePosnDurationMsec(move2) + 100;			//define a timeout in case the node is unable to enable
					}

					printf("Node \t%zi Move Done\n", iNode);
				} // for each node
				bool allNodesMoved;
				do {
					allNodesMoved = true;
					for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
						INode& theNode = myPort.Nodes(iNode);
						if (!theNode.Motion.MoveIsDone()) {
							allNodesMoved = false; // if any node is not yet moved, set to false
						}
					}
					if (!allNodesMoved) {
						myMgr->Delay(100); // delay a bit before checking again to reduce CPU usage
					}
				} while (!allNodesMoved);

				printf("Desired roll and pitch achieved.\n");

				std::cout << "All nodes successfully moved.\n";

			}
			catch (const std::exception& e) {
				std::cout << "Exception occurred: " << e.what() << std::endl;
			}
			// Wait for user to press Enter to end the program
			std::cout << "Press Enter to home trainer device..." << std::endl;
			std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Clear the buffer
			std::cin.get(); // Wait for user to press Enter

			///////////////////////////////////////////////////////////////////////////////////////
			//Home the system after moves have completed
			//////////////////////////////////////////////////////////////////////////////////////
			
			for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
				// Create a shortcut reference for a node
				INode& theNode = myPort.Nodes(iNode);

				theNode.EnableReq(false);				//Ensure Node is disabled before loading config file

				myMgr->Delay(200);


				//theNode.Setup.ConfigLoad("Config File path");


				printf("   Node[%d]: type=%d\n", int(iNode), theNode.Info.NodeType());
				printf("            userID: %s\n", theNode.Info.UserID.Value());
				printf("        FW version: %s\n", theNode.Info.FirmwareVersion.Value());
				printf("          Serial #: %d\n", theNode.Info.SerialNumber.Value());
				printf("             Model: %s\n", theNode.Info.Model.Value());

				//The following statements will attempt to enable the node.  First,
				// any shutdowns or NodeStops are cleared, finally the node is enabled
				theNode.Status.AlertsClear();					//Clear Alerts on node 
				theNode.Motion.NodeStopClear();	//Clear Nodestops on Node  				
				theNode.EnableReq(true);					//Enable node 
				//At this point the node is enabled
				printf("Node \t%zi enabled\n", iNode);
				double timeout = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT;	//define a timeout in case the node is unable to enable
				//This will loop checking on the Real time values of the node's Ready status
				while (!theNode.Motion.IsReady()) {
					if (myMgr->TimeStampMsec() > timeout) {
						printf("Error: Timed out waiting for Node %zd to enable\n", iNode);
						msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
						return -2;
					}
				}
			}
			for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
				// Create a shortcut reference for a node
				INode& theNode = myPort.Nodes(iNode);
				//At this point the Node is enabled, and we will now check to see if the Node has been homed
				//Check the Node to see if it has already been homed, 
				if (theNode.Motion.Homing.HomingValid())
				{
					if (theNode.Motion.Homing.WasHomed())
					{
						printf("Node %zd has already been homed, current position is: \t%8.0f \n", iNode, theNode.Motion.PosnMeasured.Value());
						printf("Rehoming Node... \n");
					}
					else
					{
						printf("Node [%zd] has not been homed.  Homing Node now...\n", iNode);
					}
					//Now we will home the Node
					theNode.Motion.Homing.Initiate();

					printf("Node completed homing\n");
				}
				else {
					printf("Node[%zd] has not had homing setup through ClearView.  The node will not be homed.\n", iNode);
				}
			}
			// Monitor all nodes for completion of homing
			bool allNodesHomed2;
			do {
				allNodesHomed2 = true;
				for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
					INode& theNode = myPort.Nodes(iNode);
					if (!theNode.Motion.Homing.WasHomed()) {
						allNodesHomed2 = false; // if any node is not yet homed, set to false
					}
				}
				if (!allNodesHomed2) {
					myMgr->Delay(100); // delay a bit before checking again to reduce CPU usage
				}
			} while (!allNodesHomed2);

			printf("All nodes successfully homed.\n");
			
		//////////////////////////////////////////////////////////////////////////////////////////////
		//After moves have completed Disable node, and close ports
		//////////////////////////////////////////////////////////////////////////////////////////////
			printf("Disabling nodes, and closing port\n");
			//Disable Nodes

			for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
				// Create a shortcut reference for a node
				myPort.Nodes(iNode).EnableReq(false);
			}
		}
	}
	catch (mnErr& theErr)
	{
		printf("Failed to disable Nodes n\n");
		//This statement will print the address of the error, the error code (defined by the mnErr class), 
		//as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

		msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
		return 0;  //This terminates the main program
	}

	// Close down the ports
	myMgr->PortsClose();

	msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
	return 0;			//End program
}

