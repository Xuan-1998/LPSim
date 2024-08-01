#ifndef LC_B18_GENERAL_STRUCT_H
#define LC_B18_GENERAL_STRUCT_H

namespace LC {

struct LNode {
    int data;
    LNode* next;
};

struct Node {
    int key;
    LNode* values;
    Node* next;
};

class Dict {
    Node* head;
    public:
        Dict() : head(nullptr) {}

        Node* find(int key) {
            Node* curr = head;
            while (curr) {
                if (curr->key == key) return curr;
                curr = curr->next;
            }
            return nullptr;
        }

        void insert(int key, int* values, int valuesSize) {
            Node* node = find(key);
            if (!node) {
                node = new Node;
                node->key = key;
                node->next = head;
                head = node;
            }
            node->values = nullptr;
            for (int i = 0; i < valuesSize; ++i) {
                append(key, values[i]);
            }
        }

        void append(int key, int value) {
            Node* node = find(key);
            if (!node) {
                node = new Node;
                node->key = key;
                node->next = head;
                head = node;
            }
            LNode* newNode = new LNode;
            newNode->data = value;
            newNode->next = nullptr;
            if (!node->values) {
                node->values = newNode;
            } else {
                LNode* curr = node->values;
                while (curr->next) {
                    curr = curr->next;
                }
                curr->next = newNode;
            }
        }

        int remove(int key) {
            Node** pp = &head;
            while (*pp) {
                if ((*pp)->key == key) {
                    Node* temp = *pp;
                    *pp = (*pp)->next;
                    LNode* curr = temp->values;
                    int count = 0;
                    while (curr) {
                        LNode* next = curr->next;
                        delete curr;
                        curr = next;
                        count++;
                    }
                    delete temp;
                    return count;
                }
                pp = &(*pp)->next;
            }
            return 0;
        }

        // void print() {
        //     Node* curr = head;
        //     while (curr) {
        //         std::cout << "Key: " << curr->key << " Values: ";
        //         LNode* valueCurr = curr->values;
        //         while (valueCurr) {
        //             std::cout << valueCurr->data << " ";
        //             valueCurr = valueCurr->next;
        //         }
        //         std::cout << std::endl;
        //         curr = curr->next;
        //     }
        // }
};

class LList {
    public: LNode* head; 
    LList() { 
        head = nullptr;
    }
    void append(int val) { 
        LNode* new_node = new LNode(); 
        new_node->data = val; 
        new_node->next = nullptr;
        if(head == nullptr) { 
            head = new_node; 
        } 
        else { 
            LNode* temp = head; 
            while(temp->next != nullptr) { 
                temp = temp->next; 
            } temp->next = new_node; 
        } 
    } 
    // void print() { 
    //     LNode* temp = head; 
    //     while(temp != nullptr) { 
    //         std::cout << temp->data << " "; 
    //         temp = temp->next; 
    //     } std::cout << std::endl; 
    // }
};
}
#endif // LC_B18_GENERAL_STRUCT_H