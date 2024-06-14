template<typename T>
class Quack{
  int size;
  T* data;
  
  int top;
  int bottom;
  bool empty;

public:
  Quack();
  Quack(int newSize);
  ~Quack();

  void pushTop(T newElement);
  T extractTop();
  T extractBottom();
  bool isEmpty();
  void clear();
};


template<typename T>
Quack<T>::Quack(){
  size = 20;
  data = new T[size];
  
  top = 0;
  bottom = 0;
  empty = true;
}

template<typename T>
Quack<T>::Quack(int newSize){
  size = newSize;
  data = new T[size];

  top = 0;
  bottom = 0;
  empty = true;
}

template<typename T>
Quack<T>::~Quack(){
  delete[] data;
}

template<typename T>
void Quack<T>::pushTop(T newElement){
  if(empty){
    data[top] = newElement;
    empty = false;
  }else{
    top--;
    if(top == -1) top = size-1;
    data[top] = newElement;
    if(top == bottom){
      bottom--;
      if(bottom == -1) bottom = size-1;
    }
  }
}

template<typename T>
T Quack<T>::extractTop(){
  if(top == bottom){
    T topElement = data[top];
    empty = true;
    return topElement;
  }else{
    T topElement = data[top];
    top++;
    if(top == size) top = 0;
    return topElement;
  }
}

template<typename T>
T Quack<T>::extractBottom(){
  if(top == bottom){
    T bottomElement = data[bottom];
    empty = true;
    return bottomElement;
  }else{
    T bottomElement = data[bottom];
    bottom--;
    if(bottom == -1) bottom = size-1;
    return bottomElement;
  }
}

template<typename T>
bool Quack<T>::isEmpty(){
  return empty;
}

template<typename T>
void Quack<T>::clear() {
    bottom = top;
    empty = true;
}
