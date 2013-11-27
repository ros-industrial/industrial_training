/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef FLATHEADERS
#include "simple_message/byte_array.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/log_wrapper.h"
#else
#include "byte_array.h"
#include "simple_serialize.h"
#include "log_wrapper.h"
#endif

#ifdef MOTOPLUS
#include "motoPlus.h"
#endif

#include "string.h"

namespace industrial
{
namespace byte_array
{

using namespace industrial::simple_serialize;
using namespace industrial::shared_types;
using namespace industrial::byte_array;

ByteArray::ByteArray(void)
{
  this->init();
#ifdef BYTE_SWAPPING
  LOG_COMM("Byte swapping enabled");
#endif
}

ByteArray::~ByteArray(void)
{
}

void ByteArray::init()
{
  memset(&(buffer_[0]), 0, this->MAX_SIZE);
  this->setBufferSize(0);
}

bool ByteArray::init(const char* buffer, const shared_int byte_size)
{
  bool rtn;

  if (this->MAX_SIZE >= byte_size)
  {
    LOG_COMM("Initializing buffer to size: %d", byte_size);
    this->load((void*)buffer, byte_size);
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to initialize byte array, buffer size: %u greater than max: %u",
              byte_size, this->getMaxBufferSize());
    rtn = false;
  }
  return rtn;
}

void ByteArray::copyFrom(ByteArray & buffer)
{
  if (buffer.getBufferSize() != 0)
  {
    this->setBufferSize(buffer.getBufferSize());
    memcpy(this->getRawDataPtr(), buffer.getRawDataPtr(), this->buffer_size_);
  }
  else
  {
    LOG_WARN("Byte array copy not performed, buffer to copy is empty");
  }
}


#ifdef BYTE_SWAPPING
void ByteArray::swap(void *value, shared_int byteSize)
{
  LOG_COMM("Executing byte swapping");

  LOG_COMM("Value (swapping-input): %u", (unsigned int)(*(unsigned int*)value));
  for (unsigned int i = 0; i < byteSize / 2; i++)
  {
    unsigned int endIndex = byteSize - i - 1;
    char endByte = ((char*)value)[endIndex];
    unsigned int endInt = endByte;

    unsigned int beginIndex = i;
    char beginByte = ((char*)value)[beginIndex];
    unsigned int beginInt = beginByte;

    LOG_COMM("Swap beginIndex i: %u, endIndex: %u, begin[]: %u, end[]: %u",
             beginIndex, endIndex, beginInt, endInt);
    ((char*)value)[endIndex] = beginByte;
    ((char*)value)[beginIndex] = endByte;
  }
  LOG_COMM("Value (swapping-output): %u", (unsigned int)(*(unsigned int*)value));

}
#endif

char* ByteArray::getRawDataPtr()
{
  return &this->buffer_[0];
}

/****************************************************************
 // load(*)
 //
 // Methods for loading various data types.
 //
 */
bool ByteArray::load(shared_bool value)
{
  return this->load(&value, sizeof(shared_bool));
}

bool ByteArray::load(shared_real value)
{
#ifdef BYTE_SWAPPING
  LOG_COMM("Value (loading-input): %f", value);
  this->swap(&value, sizeof(shared_real));
  LOG_COMM("Value (loading-output): %f", value);
#endif

  return this->load(&value, sizeof(shared_real));
}

bool ByteArray::load(shared_int value)
{
#ifdef BYTE_SWAPPING
  LOG_COMM("Value (loading-input): %d", value);
  this->swap(&value, sizeof(shared_int));
  LOG_COMM("Value (loading-output): %d", value);
#endif

  return this->load(&value, sizeof(shared_int));
}

bool ByteArray::load(simple_serialize::SimpleSerialize &value)
{
  LOG_COMM("Executing byte array load through simple serialize");
  return value.load(this);
}

bool ByteArray::load(ByteArray &value)
{
  LOG_COMM("Executing byte array load through byte array");
  return this->load(value.getRawDataPtr(), value.getBufferSize());
}

bool ByteArray::load(void* value, const shared_int byte_size)
{

  bool rtn;
  // Get the load pointer before extending the buffer.
  char* loadPtr;

  LOG_COMM("Executing byte array load through void*, size: %d", byte_size);
  // Check inputs
  if (NULL == value)
  {
    LOG_ERROR("NULL point passed into load method");
    return false;
  }

  loadPtr = this->getLoadPtr();

  if (this->extendBufferSize(byte_size))
  {
    memcpy(loadPtr, value, byte_size);
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to load byte array");
    rtn = false;
  }

  return rtn;
}

/****************************************************************
 // unload(*)
 //
 // Methods for unloading various data types.  Unloading data shortens
 // the internal buffer.  The resulting memory that holds the data is
 // lost.
 //
 */
bool ByteArray::unload(shared_bool & value)
{
  shared_bool rtn = this->unload(&value, sizeof(shared_bool));
  return rtn;

}

bool ByteArray::unload(shared_real &value)
{
  bool rtn = this->unload(&value, sizeof(shared_real));

#ifdef BYTE_SWAPPING
  LOG_COMM("Value (unloading-input): %f", value);
  this->swap(&value, sizeof(shared_real));
  LOG_COMM("Value (unloading-output): %f", value);
#endif

  return rtn;
}

bool ByteArray::unload(shared_int &value)
{
  bool rtn = this->unload(&value, sizeof(shared_int));

#ifdef BYTE_SWAPPING
  LOG_COMM("Value (unloading-input): %d", value);
  this->swap(&value, sizeof(shared_int));
  LOG_COMM("Value (unloading-output): %d", value);
#endif
  return rtn;
}

bool ByteArray::unload(simple_serialize::SimpleSerialize &value)
{
  LOG_COMM("Executing byte array unload through simple serialize");
  return value.unload(this);
}

bool ByteArray::unload(ByteArray &value, const shared_int byte_size)
{
  LOG_COMM("Executing byte array unload through byte array");
  char* unloadPtr = this->getUnloadPtr(byte_size);
  bool rtn;

  if (NULL != unloadPtr)
  {
    if (this->shortenBufferSize(byte_size))
    {

      rtn = value.load(unloadPtr, byte_size);
      rtn = true;
    }
    else
    {
      LOG_ERROR("Failed to shorten array");
      rtn = false;
    }
  }
  else
  {
    LOG_ERROR("Unload pointer returned NULL");
    rtn = false;
  }

  return rtn;
}

bool ByteArray::unload(void* value, shared_int byteSize)
{
  bool rtn;
  char* unloadPtr;

  LOG_COMM("Executing byte array unload through void*, size: %d", byteSize);
  // Check inputs
  if (NULL == value)
  {
    LOG_ERROR("NULL point passed into unload method");
    return false;
  }

  unloadPtr = this->getUnloadPtr(byteSize);

  if (NULL != unloadPtr)
  {

    if (this->shortenBufferSize(byteSize))
    {
      memcpy(value, unloadPtr, byteSize);
      rtn = true;
    }
    else
    {
      LOG_ERROR("Failed to shorten array");
      rtn = false;
    }
  }
  else
  {
    LOG_ERROR("Unload pointer returned NULL");
    rtn = false;
  }

  return rtn;
}



/****************************************************************
 // unloadFront(*)
 //
 // Methods for unloading various data types.  Unloading data shortens
 // the internal buffer and requires a memmove.  These functions should
 // be used sparingly, as they are expensive.
 //
 */
bool ByteArray::unloadFront(industrial::shared_types::shared_real &value)
{
  bool rtn = this->unloadFront(&value, sizeof(shared_real));

#ifdef BYTE_SWAPPING
  LOG_COMM("Value (unloading-input): %f", value);
  this->swap(&value, sizeof(shared_real));
  LOG_COMM("Value (unloading-output): %f", value);
#endif
  return rtn;
}

bool ByteArray::unloadFront(industrial::shared_types::shared_int &value)
{
  bool rtn = this->unloadFront(&value, sizeof(shared_int));

#ifdef BYTE_SWAPPING
  LOG_COMM("Value (unloading-input): %d", value);
  this->swap(&value, sizeof(shared_int));
  LOG_COMM("Value (unloading-output): %d", value);
#endif
  return rtn;
}
bool ByteArray::unloadFront(void* value, const industrial::shared_types::shared_int byteSize)
{
  bool rtn;
  char* unloadPtr = NULL;
  char* nextPtr = NULL;
  shared_int sizeRemain;

  // Check inputs
  if (NULL == value)
  {
    LOG_ERROR("NULL point passed into unload method");
    return false;
  }

  unloadPtr = &this->buffer_[0];

  if (NULL != unloadPtr)
  {
    nextPtr = unloadPtr + byteSize;
    sizeRemain = this->getBufferSize() - byteSize;

    LOG_DEBUG("Unloading: %d bytes, %d bytes remain", byteSize, sizeRemain);
    if (this->shortenBufferSize(byteSize))
    {
      LOG_COMM("Preparing to copy value");
      memcpy(value, unloadPtr, byteSize);
      LOG_COMM("Value is unloaded, performing move");
      memmove(unloadPtr, nextPtr, sizeRemain);
      LOG_COMM("Move operation completed");
      rtn = true;
    }
    else
    {
      LOG_ERROR("Failed to shorten array");
      rtn = false;
    }
  }
  else
  {
    LOG_ERROR("Unload pointer returned NULL");
    rtn = false;
  }

  return rtn;
}

unsigned int ByteArray::getBufferSize()
{
  return this->buffer_size_;
}

unsigned int ByteArray::getMaxBufferSize()
{
  return this->MAX_SIZE;
}


bool ByteArray::isByteSwapEnabled()
{
#ifdef BYTE_SWAPPING
  return true;
#endif
  return false;
}

bool ByteArray::setBufferSize(shared_int size)
{
  bool rtn;

  if (this->MAX_SIZE >= size)
  {
    this->buffer_size_ = size;
    rtn = true;
  }
  else
  {
    LOG_ERROR("Set buffer size: %u, larger than MAX:, %u", size, this->MAX_SIZE);
    rtn = false;
  }

  return rtn;

}

bool ByteArray::extendBufferSize(shared_int size)
{
  unsigned int newSize;

  newSize = this->getBufferSize() + size;
  return this->setBufferSize(newSize);

}

bool ByteArray::shortenBufferSize(shared_int size)
{
  unsigned int newSize;
  bool rtn;

  // If the buffer is not larger than the size it is shortened by
  // we fail.  This is checked here (as opposed to setBufferSize)
  // because setBufferSize assumes a unsigned argument and therefore
  // wouldn't catch a negative size.
  if (size <= (shared_int)this->getBufferSize())
  {
    newSize = this->getBufferSize() - size;
    rtn = this->setBufferSize(newSize);
  }
  else
  {
    LOG_ERROR("Failed to shorten buffer by %u bytes, buffer too small, %u bytes", size, this->getBufferSize());
    rtn = false;
  }

  return rtn;

}

char* ByteArray::getLoadPtr()
{

  return &this->buffer_[this->buffer_size_];
}

char* ByteArray::getUnloadPtr(shared_int byteSize)
{
  char* rtn;

  if (byteSize <= (shared_int)this->getBufferSize())
  {
    rtn = this->getLoadPtr() - byteSize;
  }
  else
  {
    LOG_ERROR("Get unload pointer failed, buffer size: %d, smaller than byte size: %d",
              this->getBufferSize(), byteSize);
    rtn = NULL;
  }

  return rtn;
}

} // namespace byte_array
} // namespace industrial
