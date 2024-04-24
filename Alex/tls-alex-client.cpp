#include "make_tls_client.h" // Routines to create a TLS client
#include "netconstants.h" // Network packet types
#include "constants.h" // Packet types, error codes, etc.

static volatile int networkActive=0; // To check if network is running.

// Parameters and Filenames for TLS Handshake
#define SERVER_NAME "192.168.139.41" // R-Pi IP address
#define PORT_NUM 5001 // Port number
#define CA_CERT_FNAME "signing.pem" // CA certificate name
#define CLIENT_CERT_FNAME "laptop.crt" // PC's certificate
#define CLIENT_KEY_FNAME "laptop.key" // PC's private key
#define SERVER_NAME_ON_CERT "signer.com" // R-Pi's name


/*
	Network Routines with R-Pi 
*/
void handleError(const char *buffer)
{
	switch(buffer[1])
	{
		case RESP_OK:
			printf("Command / Status OK\n");
			break;

		case RESP_BAD_PACKET:
			printf("BAD MAGIC NUMBER FROM ARDUINO\n");
			break;

		case RESP_BAD_CHECKSUM:
			printf("BAD CHECKSUM FROM ARDUINO\n");
			break;

		case RESP_BAD_COMMAND:
			printf("PI SENT BAD COMMAND TO ARDUINO\n");
			break;

		case RESP_BAD_RESPONSE:
			printf("PI GOT BAD RESPONSE FROM ARDUINO\n");
			break;

		default:
			printf("PI IS CONFUSED!\n");
	}
}

void handleStatus(const char *buffer)
{
	int32_t data[16];
	memcpy(data, &buffer[1], sizeof(data));

	printf("\n ------- ALEX STATUS REPORT ------- \n\n");
	printf("Left Forward Ticks:\t\t%d\n", data[0]);
	printf("Left Reverse Ticks:\t\t%d\n", data[1]);
	printf("Left Forward Ticks Turns:\t%d\n", data[2]);
	printf("Left Reverse Ticks Turns:\t%d\n", data[3]);
	printf("Forward Distance:\t\t%d\n", data[4]);
	printf("Reverse Distance:\t\t%d\n", data[5]);
	printf("Red:\t\t%d\n", data[6]);
	printf("Green:\t\t%d\n", data[7]);
	printf("Blue:\t\t%d\n", data[8]);
	printf("Colour:\t\t%d\n", data[9]);
	printf("Distance:\t\t%d\n", data[10]);
	printf("\n---------------------------------------\n\n");
}

// For debugging, not used in mission
void handleMessage(const char *buffer)
{
	printf("MESSAGE FROM ALEX: %s\n", &buffer[1]);
}

void handleNetwork(const char *buffer, int len)
{
	int type = buffer[0]; // The first byte is the packet type
	switch(type)
	{
		case NET_ERROR_PACKET:
		handleError(buffer);
		break;

		case NET_STATUS_PACKET:
		handleStatus(buffer);
		break;

		case NET_MESSAGE_PACKET:
		handleMessage(buffer);
		break;

		case NET_COMMAND_PACKET:
		handleCommand(buffer);
		break;
	}
}

void sendData(void *conn, const char *buffer, int len)
{
	int c;
	printf("\nSENDING %d BYTES DATA\n\n", len);

	if(networkActive)
	{
        int count = sslWrite(conn,buffer,len);
		if(count<0){
		   perror("Error writing to server");
		}
		networkActive = (c > 0);
	}
}

void *readerThread(void *conn)
{
	char buffer[128];
	int len;

	while(networkActive)
	{
        len = sslRead(conn,buffer,sizeof(buffer));
		if(len<0){
		    perror("Error reading from server");
		}
        printf("read %d bytes from server.\n", len);

		networkActive = (len > 0);
		if(networkActive)
			handleNetwork(buffer, len);
	}
	printf("Exiting network listener thread\n");

    stopClient();
    EXIT_THREAD(conn);
    return NULL;
}

void connectToServer(const char *serverName, int portNum)
{
     createClient(SERVER_NAME,PORT_NUM,1,CA_CERT_FNAME,SERVER_NAME_ON_CERT,1,CLIENT_CERT_FNAME,CLIENT_KEY_FNAME, readerThread,writerThread);
}


/*
	Scanning Operators' Command
*/
void flushInput()
{
	char c;
	while((c = getchar()) != '\n' && c != EOF);
}

void getParams(int32_t *params)
{
	printf("Enter distance/angle in cm/degrees (e.g. 50) and power in %% (e.g. 75) separated by space.\n");
	printf("E.g. 50 75 means go at 50 cm at 75%% power for forward/backward, or 50 degrees left or right turn at 75%%  power\n");
	scanf("%d", &params[0]);
	params[1] = 100;
	flushInput();
}

void *writerThread(void *conn)
{
	int quit=0;

	while(!quit)
	{
		char ch;
		printf("Command (f=forward, b=reverse, l=turn left, r=turn right, s=stop, c=clear stats, g=get stats q=exit)\n");
		scanf("%c", &ch);

		// Purge extraneous characters from input stream
		flushInput();

		char buffer[10];
		int32_t params[2];

		buffer[0] = NET_COMMAND_PACKET;
		switch(ch)
		{
			case 'f':
			case 'F':
			case 'b':
			case 'B':
			case 'l':
			case 'L':
			case 'r':
			case 'R':
						getParams(params);
						buffer[1] = ch;
						memcpy(&buffer[2], params, sizeof(params));
						sendData(conn, buffer, sizeof(buffer));
						break;
			case 's':
			case 'S':
			case 'g':
			case 'G':
					params[0]=0;
					params[1]=0;
					memcpy(&buffer[2], params, sizeof(params));
					buffer[1] = ch;
					sendData(conn, buffer, sizeof(buffer));
					break;
			case 'q':
			case 'Q':
				quit=1;
				break;
			default:
				printf("BAD COMMAND\n");
		}
	}
	printf("Exiting keyboard thread\n");
    stopClient();
    EXIT_THREAD(conn);
    return NULL;
}


/*
	Main Function
*/
int main(int ac, char **av)
{
	if(ac != 3)
	{
		fprintf(stderr, "\n\n%s <IP address> <Port Number>\n\n", av[0]);
		exit(-1);
	}

    networkActive = 1;
    connectToServer(av[1], atoi(av[2]));

    while(client_is_running()); // Prevent main from exiting

	printf("\nMAIN exiting\n\n");
}
