RTDB is an open source middleware, developed to help in the task
of programming teams of mobile robots.


Check for last updates:
  http://code.google.com/p/rtdb/


The RTDB middleware includes:
- A replicated blackboard called RTDB, which olds the state data
  of each agent (robot) together with local images of the state data
  shared by other team members. This way, all the access to remote
  information is made by local queries avoiding communication delays.
- A communication system, build on top of WiFi IEEE802.11, that
  triggers the required transactions, in the background. The
  communication is similar to a TDMA scheme, but without the need of
  clock synchronization. All the transmissions between the team are
  separeted as much as possible avoiding collisions in intra-team
  communications.


Project folders:
- bin: include the binary rtdb_parser (not distributed as open source)
  and comm binary, after compilation;
- comm: files related with the communication system. 
- config: rtdb configuration files
- example: simple example
- lib: rtdb lib to be used with team code
- rtdb: rtdb blackboard files


Version history:
  3.0 - Only dynamic ZERO makes the Adaptation.
        The other agents transmits at fixed delays after ZERO

  2.1 - Reconfiguration of the round to be divided dinamically by
        the running agents
  2.0 - Rebuild to be used on common Linux.

  1.0 - Build on the top of a RTAI kernel.


ToDo:
- Unique configuration file based on XML language, with open-source
  parser.


Developed by:
  Frederico Miguel Santos <frederico.miguel.santos@gmail.com>
  CAMBADA robotic soccer team - University of Aveiro
  http://www.ieeta.pt/atri/cambada
