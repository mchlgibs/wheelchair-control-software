#include <iostream>
#include <cstdio>
#include "pubSysCls.h"	
#include <cassert>

using namespace sFnd;

void waitForEnter() {
    (void)getchar();
}

int main()
{
    std::cout << "Starting. Press Enter to continue.\n";
    waitForEnter();

    SysManager* myMgr = SysManager::Instance();

    try {
        std::vector<std::string> comHubPorts;
        SysManager::FindComHubPorts(comHubPorts);

        printf("Found %zd SC Hubs\n", comHubPorts.size());

        assert(comHubPorts.size() <= NET_CONTROLLER_MAX);

        std::cout << "Here are the ports I found:" << std::endl;
        for (int i = 0; i < comHubPorts.size(); i++) {
            std::cout << "\t" << comHubPorts[i] << std::endl;
        }
        std::cout << std::endl;

/**/
        // Open all the ports
        myMgr->PortsOpen(comHubPorts.size());

        for (int i = 0; i < comHubPorts.size(); i++) {
            myMgr->ComHubPort(i, comHubPorts[i].c_str());
            IPort& port = myMgr->Ports(i);
            printf("Port[%d]: state=%d, nodes=%d\n", port.NetNumber(), port.OpenState(), port.NodeCount());

            for (size_t iNode = 0; iNode < port.NodeCount(); iNode++) {
                // Create a shortcut reference for a node
                INode& theNode = port.Nodes(iNode);

                theNode.EnableReq(false);				//Ensure Node is disabled before loading config file
                myMgr->Delay(200);

                printf("\tNode[%d]:\n", int(iNode));
                printf("\t\ttype:       %d\n", theNode.Info.NodeType());
                printf("\t\tuserID:     %s\n", theNode.Info.UserID.Value());
                printf("\t\tFW version: %s\n", theNode.Info.FirmwareVersion.Value());
                printf("\t\tSerial #:   %d\n", theNode.Info.SerialNumber.Value());
                printf("\t\tModel:      %s\n", theNode.Info.Model.Value());
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
    catch (mnErr& e) {
        printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", e.TheAddr, e.ErrorCode, e.ErrorMsg);
        std::cout << "Press Enter to continue.\n";
        waitForEnter();
    }

    myMgr->PortsClose();
    std::cout << "All done. Press Enter to continue.\n";
    waitForEnter();
}
