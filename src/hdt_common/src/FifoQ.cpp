#include <stdio.h>
#include <string.h>

#include "FifoQ.h"

FifoQ::FifoQ(int objSize, int len, Mode  m){
	mObjSize = objSize;
	mLength = len;
	mMode = m;
  	mHead = mTail = mSize = 0;
//	mFifoPtr = (char *)malloc(len * objSize);
	mFifoPtr = new char[len * objSize];
	sem_init(&mSemaphore, 1, 1);
}

int FifoQ::Put(void *obj){
  int ret = -1;

  sem_wait(&mSemaphore);

  if(mSize < mLength){
	memcpy(mFifoPtr + mHead*mObjSize, obj, mObjSize);
    ++mHead %= mLength;
    mSize++;
    ret = 1;
  }
  else{
    if(mMode == DISCARD){
      ret = -1;
    }
    else{
	  memcpy(mFifoPtr + mHead*mObjSize, obj, mObjSize);
      ++mHead %= mLength;
      mTail++;
      mTail %= mLength;
      mSize = mLength;
      ret = 1;
    }
  }

  sem_post(&mSemaphore);

  return ret;
}

int FifoQ::Get(void *obj){
  int ret = -1;

  sem_wait(&mSemaphore);

  if(mSize > 0){
	memcpy(obj, mFifoPtr + mTail*mObjSize, mObjSize);
    ++mTail %= mLength;
    mSize--;
    ret = 1;
  }
  else{
    ret = -1;
  }

  sem_post(&mSemaphore);

  return ret;
}

int FifoQ::Read(void *obj, int index){
	int ret = -1;

	sem_wait(&mSemaphore);

//	printf("mHead = %d, mTail = %d\n", mHead, mTail);

 	if(mSize > 0){
		if(index > mSize){
			ret = -1;
		}
		else{
			mIndex = mTail + index;
			mIndex %= mLength;
			memcpy(obj, mFifoPtr + mIndex*mObjSize, mObjSize);
//			obj = &mFifoPtr[mIndex];
			ret = 1;
		}
	}
	else{
		ret = -1;
	}

 	sem_post(&mSemaphore);

	return ret;
}

