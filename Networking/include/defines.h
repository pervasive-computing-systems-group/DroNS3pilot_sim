// Debug flag
#define DEBUG	0
// Universal port
#define DATA_PORT	"8080"

// enum for message type
enum E_MessageType {REQUEST, RESPONSE, BAD_MESSAGE};

// struct passed back and forth
struct packet_t {
	int id;
	E_MessageType msg_type;
	int bytes_to_send;

	packet_t() {
		id = -1;
		msg_type = E_MessageType::BAD_MESSAGE;
		bytes_to_send = 0;
	}
};

// Prints debugging message dbgString if DEBUG flag is set
void debugPrint(const char *dbgString) {
	if(DEBUG) {
		puts(dbgString);
	}
}
