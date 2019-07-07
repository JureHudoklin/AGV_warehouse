

template <class T>
T getSign(T number) {
    if (number < 0) {
        return -1;
    } else if(number > 0){
        return 1;
    } else {
		return 0;
	}
}


template <class T>
T limitNumber(T number, T max) {
    if (number > max) {
        return max;
    } else if (number < -max) {
        return -max;
    } else {
        return number;
    }
}

template <class T>
int getMinIndex(T* number, int size) {

    int index = 0;
    T a = number[0];
    for(int i = 1; i<size; i++) {
        if (number[i]<a) {
            a = number[i];
            index = i;
        }
    }
    return index;
}
