 
 

// A linked list (LL) node to store a queue entry
struct QNode
{
    float key;
    struct QNode *next;
};
 
// The queue, front stores the front node of LL and rear stores ths
// last node of LL
struct Queue
{
    struct QNode *front, *rear;
};

struct QNode* newNode(float k);
struct Queue *createQueue(void);
void enQueue(struct Queue *q, float k);
struct QNode *deQueue(struct Queue *q);
