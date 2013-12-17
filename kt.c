#define maxkeys 12
#define PORTDIR DDRA
#define PORTDATA PORTA  
#define PORTIN PINA

#define t1 20

unsigned char PushFlag;
unsigned char PushState;

#define NoPush 1
#define MaybePush 2
#define Pushed 4
#define MaybeNoPush 4

unsigned char key;
unsigned char butnum;

unsigned char keytbl[16] = 
{
	0x77, 0x7b, 0x7d, 0xb7,
	0xbb, 0xbd, 0xd7, 0xdb,
	0xdd, 0xe7, 0xeb, 0xed,
	0xff, 0xff, 0xff, 0xff
};

unsigned int mem[12] = 
{
	0,0,0,0,0,0,0,0,0,0,0,0
};

unsigned int mem_index;
unsigned high_freq[12] = 
{ 
	1209, 1336, 1477,
	1209, 1336, 1477,
	1209, 1336, 1477,
	1209, 1336, 1477
};

unsigned int low_freq[12] = 
{ 
	697, 697, 697, 
	770, 770, 770, 
	852, 852, 852, 
	941, 941, 941
};

void task1(void){
	time1 = 0;

	//PORTDIR = 0x0f;
	//PORTDATA = 0xf0;
	_delay_us(5);
	key = PORTIN;


}