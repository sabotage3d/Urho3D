//
// Copyright (c) 2008-2015 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include "../Precompiled.h"

#include "../Graphics/Geometry.h"
#include "../Graphics/Graphics.h"
#include "../Graphics/IndexBuffer.h"
#include "../Graphics/VertexBuffer.h"
#include "../IO/Log.h"
#include "../Math/Ray.h"

#include "../DebugNew.h"

//Static batching
#include "../IO/VectorBuffer.h"


#include <iostream>

namespace Urho3D
{

Geometry::Geometry(Context* context) :
    Object(context),
    primitiveType_(TRIANGLE_LIST),
    indexStart_(0),
    indexCount_(0),
    vertexStart_(0),
    vertexCount_(0),
    positionBufferIndex_(M_MAX_UNSIGNED),
    rawVertexSize_(0),
    rawElementMask_(0),
    rawIndexSize_(0),
    lodDistance_(0.0f)
{
    SetNumVertexBuffers(1);
}

Geometry::~Geometry()
{
}

bool Geometry::SetNumVertexBuffers(unsigned num)
{
    if (num >= MAX_VERTEX_STREAMS)
    {
        URHO3D_LOGERROR("Too many vertex streams");
        return false;
    }

    unsigned oldSize = vertexBuffers_.Size();
    vertexBuffers_.Resize(num);
    elementMasks_.Resize(num);

    for (unsigned i = oldSize; i < num; ++i)
        elementMasks_[i] = MASK_NONE;

    GetPositionBufferIndex();
    return true;
}

bool Geometry::SetVertexBuffer(unsigned index, VertexBuffer* buffer, unsigned elementMask)
{
    if (index >= vertexBuffers_.Size())
    {
        URHO3D_LOGERROR("Stream index out of bounds");
        return false;
    }

    vertexBuffers_[index] = buffer;

    if (buffer)
    {
        if (elementMask == MASK_DEFAULT)
            elementMasks_[index] = buffer->GetElementMask();
        else
            elementMasks_[index] = elementMask;
    }

    GetPositionBufferIndex();
    return true;
}

void Geometry::SetIndexBuffer(IndexBuffer* buffer)
{
    indexBuffer_ = buffer;
}

bool Geometry::SetDrawRange(PrimitiveType type, unsigned indexStart, unsigned indexCount, bool getUsedVertexRange)
{
    if (!indexBuffer_ && !rawIndexData_)
    {
        URHO3D_LOGERROR("Null index buffer and no raw index data, can not define indexed draw range");
        return false;
    }
    if (indexBuffer_ && indexStart + indexCount > indexBuffer_->GetIndexCount())
    {
        URHO3D_LOGERROR("Illegal draw range " + String(indexStart) + " to " + String(indexStart + indexCount - 1) + ", index buffer has " +
                 String(indexBuffer_->GetIndexCount()) + " indices");
        return false;
    }

    primitiveType_ = type;
    indexStart_ = indexStart;
    indexCount_ = indexCount;

    // Get min.vertex index and num of vertices from index buffer. If it fails, use full range as fallback
    if (indexCount)
    {
        vertexStart_ = 0;
        vertexCount_ = vertexBuffers_[0] ? vertexBuffers_[0]->GetVertexCount() : 0;

        if (getUsedVertexRange && indexBuffer_)
            indexBuffer_->GetUsedVertexRange(indexStart_, indexCount_, vertexStart_, vertexCount_);
    }
    else
    {
        vertexStart_ = 0;
        vertexCount_ = 0;
    }

    return true;
}

bool Geometry::SetDrawRange(PrimitiveType type, unsigned indexStart, unsigned indexCount, unsigned minVertex, unsigned vertexCount,
    bool checkIllegal)
{
    if (indexBuffer_)
    {
        // We can allow setting an illegal draw range now if the caller guarantees to resize / fill the buffer later
        if (checkIllegal && indexStart + indexCount > indexBuffer_->GetIndexCount())
        {
            URHO3D_LOGERROR("Illegal draw range " + String(indexStart) + " to " + String(indexStart + indexCount - 1) +
                     ", index buffer has " + String(indexBuffer_->GetIndexCount()) + " indices");
            return false;
        }
    }
    else if (!rawIndexData_)
    {
        indexStart = 0;
        indexCount = 0;
    }

    primitiveType_ = type;
    indexStart_ = indexStart;
    indexCount_ = indexCount;
    vertexStart_ = minVertex;
    vertexCount_ = vertexCount;

    return true;
}

void Geometry::SetLodDistance(float distance)
{
    if (distance < 0.0f)
        distance = 0.0f;

    lodDistance_ = distance;
}

void Geometry::SetRawVertexData(SharedArrayPtr<unsigned char> data, unsigned vertexSize, unsigned elementMask)
{
    rawVertexData_ = data;
    rawVertexSize_ = vertexSize;
    rawElementMask_ = elementMask;
}

void Geometry::SetRawIndexData(SharedArrayPtr<unsigned char> data, unsigned indexSize)
{
    rawIndexData_ = data;
    rawIndexSize_ = indexSize;
}

void Geometry::Draw(Graphics* graphics)
{
    if (indexBuffer_ && indexCount_ > 0)
    {
        graphics->SetIndexBuffer(indexBuffer_);
        graphics->SetVertexBuffers(vertexBuffers_, elementMasks_);
        graphics->Draw(primitiveType_, indexStart_, indexCount_, vertexStart_, vertexCount_);
    }
    else if (vertexCount_ > 0)
    {
        graphics->SetVertexBuffers(vertexBuffers_, elementMasks_);
        graphics->Draw(primitiveType_, vertexStart_, vertexCount_);
    }
}

VertexBuffer* Geometry::GetVertexBuffer(unsigned index) const
{
    return index < vertexBuffers_.Size() ? vertexBuffers_[index] : (VertexBuffer*)0;
}

unsigned Geometry::GetVertexElementMask(unsigned index) const
{
    return index < elementMasks_.Size() ? elementMasks_[index] : 0;
}

unsigned short Geometry::GetBufferHash() const
{
    unsigned short hash = 0;

    for (unsigned i = 0; i < vertexBuffers_.Size(); ++i)
    {
        VertexBuffer* vBuf = vertexBuffers_[i];
        hash += *((unsigned short*)&vBuf);
    }

    IndexBuffer* iBuf = indexBuffer_;
    hash += *((unsigned short*)&iBuf);

    return hash;
}

void Geometry::GetRawData(const unsigned char*& vertexData, unsigned& vertexSize, const unsigned char*& indexData,
    unsigned& indexSize, unsigned& elementMask) const
{
    if (rawVertexData_)
    {
        vertexData = rawVertexData_;
        vertexSize = rawVertexSize_;
        elementMask = rawElementMask_;
    }
    else
    {
        if (positionBufferIndex_ < vertexBuffers_.Size() && vertexBuffers_[positionBufferIndex_])
        {
            vertexData = vertexBuffers_[positionBufferIndex_]->GetShadowData();
            if (vertexData)
            {
                vertexSize = vertexBuffers_[positionBufferIndex_]->GetVertexSize();
                elementMask = vertexBuffers_[positionBufferIndex_]->GetElementMask();
            }
            else
            {
                vertexSize = 0;
                elementMask = 0;
            }
        }
        else
        {
            vertexData = 0;
            vertexSize = 0;
            elementMask = 0;
        }
    }

    if (rawIndexData_)
    {
        indexData = rawIndexData_;
        indexSize = rawIndexSize_;
    }
    else
    {
        if (indexBuffer_)
        {
            indexData = indexBuffer_->GetShadowData();
            if (indexData)
                indexSize = indexBuffer_->GetIndexSize();
            else
                indexSize = 0;
        }
        else
        {
            indexData = 0;
            indexSize = 0;
        }
    }
}

void Geometry::GetRawDataShared(SharedArrayPtr<unsigned char>& vertexData, unsigned& vertexSize,
    SharedArrayPtr<unsigned char>& indexData, unsigned& indexSize, unsigned& elementMask) const
{
    if (rawVertexData_)
    {
        vertexData = rawVertexData_;
        vertexSize = rawVertexSize_;
        elementMask = rawElementMask_;
    }
    else
    {
        if (positionBufferIndex_ < vertexBuffers_.Size() && vertexBuffers_[positionBufferIndex_])
        {
            vertexData = vertexBuffers_[positionBufferIndex_]->GetShadowDataShared();
            if (vertexData)
            {
                vertexSize = vertexBuffers_[positionBufferIndex_]->GetVertexSize();
                elementMask = vertexBuffers_[positionBufferIndex_]->GetElementMask();
            }
            else
            {
                vertexSize = 0;
                elementMask = 0;
            }
        }
        else
        {
            vertexData = 0;
            vertexSize = 0;
            elementMask = 0;
        }
    }

    if (rawIndexData_)
    {
        indexData = rawIndexData_;
        indexSize = rawIndexSize_;
    }
    else
    {
        if (indexBuffer_)
        {
            indexData = indexBuffer_->GetShadowDataShared();
            if (indexData)
                indexSize = indexBuffer_->GetIndexSize();
            else
                indexSize = 0;
        }
        else
        {
            indexData = 0;
            indexSize = 0;
        }
    }
}

float Geometry::GetHitDistance(const Ray& ray, Vector3* outNormal, Vector2* outUV) const
{
    const unsigned char* vertexData;
    const unsigned char* indexData;
    unsigned vertexSize;
    unsigned indexSize;
    unsigned elementMask;
    unsigned uvOffset = 0;

    GetRawData(vertexData, vertexSize, indexData, indexSize, elementMask);

    if (vertexData)
    {
        if (outUV)
        {
            if ((elementMask & MASK_TEXCOORD1) == 0)
            {
                // requested UV output, but no texture data in vertex buffer
                URHO3D_LOGWARNING("Illegal GetHitDistance call: UV return requested on vertex buffer without UV coords");
                *outUV = Vector2::ZERO;
                outUV = 0;
            }
            else
                uvOffset = VertexBuffer::GetElementOffset(elementMask, ELEMENT_TEXCOORD1);
        }

        return indexData ? ray.HitDistance(vertexData, vertexSize, indexData, indexSize, indexStart_, indexCount_, outNormal, outUV,
            uvOffset) :
               ray.HitDistance(vertexData, vertexSize, vertexStart_, vertexCount_, outNormal, outUV, uvOffset);
    }

    return M_INFINITY;
}

bool Geometry::IsInside(const Ray& ray) const
{
    const unsigned char* vertexData;
    const unsigned char* indexData;
    unsigned vertexSize;
    unsigned indexSize;
    unsigned elementMask;

    GetRawData(vertexData, vertexSize, indexData, indexSize, elementMask);

    return vertexData ? (indexData ? ray.InsideGeometry(vertexData, vertexSize, indexData, indexSize, indexStart_, indexCount_) :
                         ray.InsideGeometry(vertexData, vertexSize, vertexStart_, vertexCount_)) : false;
}

void Geometry::GetPositionBufferIndex()
{
    for (unsigned i = 0; i < vertexBuffers_.Size(); ++i)
    {
        if (vertexBuffers_[i] && vertexBuffers_[i]->GetElementMask() & MASK_POSITION)
        {
            positionBufferIndex_ = i;
            return;
        }
    }

    // No vertex buffer with positions
    positionBufferIndex_ = M_MAX_UNSIGNED;
}
    
//Static Batching
Geometry* Geometry::CreatePretransformedList(PODVector<Matrix3x4> transforms, bool applyTransforms) const
{
    // Perform easy early-out tests
    if (transforms.Size() == 0)
        return 0;
    if (vertexBuffers_.Size() == 0)
    {
        URHO3D_LOGERROR("Unable to create pretransformed geometry for empty geometry");
        return 0;
    }
    if (vertexBuffers_.Size() > 1)
    {
        URHO3D_LOGERROR("Geometries with multiple buffers may not be merged into pretransformed lists.");
        return 0;
    }

    // Check whether this is likely to be valid geometry.
    const unsigned srcVertexCt = GetVertexCount();
    const unsigned srcVertexSize = vertexBuffers_[0]->GetVertexSize();
    if (srcVertexSize == 0)
    {
        URHO3D_LOGERROR("VertexBuffer does not contain data: vertex size = 0");
        return 0;
    }
    if (srcVertexCt == 0)
    {
        URHO3D_LOGERROR("VertexBuffer does not contain data: vertex count = 0");
        return 0;
    }

    // Grab source element mask and also make grab the shadow data and verify it's fine
    const unsigned srcElemMask = vertexBuffers_[0]->GetElementMask();
    const unsigned char* srcVertData = vertexBuffers_[0]->GetShadowData();
    const unsigned char* srcIndexData = indexBuffer_ ? indexBuffer_->GetShadowData() : 0;
    if (!srcVertData)
    {
        URHO3D_LOGERROR("No shadow data for concatenating geometry");
        return 0;
    }

    // Prepare to construct the vertex buffer
    //Geometry* combinedGeometry = new Geometry(context_);
    SharedPtr<Geometry> combinedGeometry(new Geometry(context_));
    //VertexBuffer* combinedVertBuffer = new VertexBuffer(context_, false);
    SharedPtr<VertexBuffer> combinedVertBuffer(new VertexBuffer(context_));
    std::cout << "Vertex Count: " << GetVertexCount() << std::endl;
    std::cout << "Transform Size: " << transforms.Size() << std::endl;

    combinedVertBuffer->SetShadowed(true);
    combinedVertBuffer->SetSize(GetVertexCount() * transforms.Size(), srcElemMask, true);
    //combinedVertBuffer->SetSize(this->GetVertexCount() * transforms.Size(), MASK_POSITION|MASK_NORMAL);
    VectorBuffer vertexData;
    
    // Initial write of the vertex data as received
    for (unsigned objIndex = 0; objIndex < transforms.Size(); ++objIndex)
        vertexData.Write(srcVertData + GetVertexStart() * srcVertexSize, srcVertexCt * srcVertexSize);

    if (applyTransforms)
    {
        // Grab the element offsets to see what we need to transform
        const unsigned posOffset = VertexBuffer::GetElementOffset(srcElemMask, ELEMENT_POSITION);
        const unsigned normOffset = srcElemMask & MASK_NORMAL ? VertexBuffer::GetElementOffset(srcElemMask, ELEMENT_NORMAL) : -1;
        const unsigned tangentOffset = srcElemMask & MASK_TANGENT ? VertexBuffer::GetElementOffset(srcElemMask, ELEMENT_TANGENT) : -1;
        const unsigned cubeCoord1Offset = srcElemMask & MASK_CUBETEXCOORD1 ? VertexBuffer::GetElementOffset(srcElemMask, ELEMENT_CUBETEXCOORD1) : -1;
        const unsigned cubeCoord2Offset = srcElemMask & MASK_CUBETEXCOORD2 ? VertexBuffer::GetElementOffset(srcElemMask, ELEMENT_CUBETEXCOORD2) : -1;

        // Transform the vertex elements that need to be transformed
        for (unsigned objIndex = 0; objIndex < transforms.Size(); ++objIndex)
        {
            const Matrix3x4 transform = transforms[objIndex];
            const Matrix3 rotationMat = transform.RotationMatrix();

            // (objIndex * srcVertexSize * srcVertexCt) == byte offset into the current object
            // (vertIndex * srcVertexSize) + offsetName) == byte offset into where the current element is
            #define VERT_ASSIGNMENT(vecType, offsetName, transMat) *((vecType*)(vertexData.GetModifiableData() + (objIndex * srcVertexSize * srcVertexCt) + (vertIndex * srcVertexSize) + offsetName)) = transMat * *((vecType*)(vertexData.GetData() + (objIndex * srcVertexSize * srcVertexCt) + (vertIndex * srcVertexSize) + offsetName));
            for (unsigned vertIndex = 0; vertIndex < GetVertexCount(); ++vertIndex)
            {
                if (posOffset != M_MAX_UNSIGNED)
                    VERT_ASSIGNMENT(Vector3, posOffset, transform);
                if (normOffset != M_MAX_UNSIGNED)
                    VERT_ASSIGNMENT(Vector3, normOffset, rotationMat);
                if (tangentOffset != M_MAX_UNSIGNED)
                    VERT_ASSIGNMENT(Vector3, tangentOffset, rotationMat);
                if (cubeCoord1Offset != M_MAX_UNSIGNED)
                    VERT_ASSIGNMENT(Vector3, cubeCoord1Offset, rotationMat);
                if (cubeCoord2Offset != M_MAX_UNSIGNED)
                    VERT_ASSIGNMENT(Vector3, cubeCoord2Offset, rotationMat);
            }
            #undef VERT_ASSIGNMENT
        }
    }

    // Setup the combined geometry
    combinedVertBuffer->SetData(vertexData.GetData());
    combinedGeometry->SetNumVertexBuffers(1);
    combinedGeometry->SetVertexBuffer(0, combinedVertBuffer, srcElemMask);
    
    // If there are indices, then construct the index buffer
    unsigned indexCount = 0;
    if (indexBuffer_.NotNull() && srcIndexData)
    {
        // Prepare and size the buffers
        IndexBuffer* combinedIndexBuffer = new IndexBuffer(context_, false);
        VectorBuffer indexData;
        indexCount = GetIndexCount() * transforms.Size();
        
        #define M_MAX_USHORT 0xFFFF
        const bool largeIndices = indexCount > M_MAX_USHORT;
        #undef M_MAX_USHORT

        // Setup buffer
        if (largeIndices)
        {
            combinedIndexBuffer->SetSize(indexCount, true, false);
            indexData.Resize(indexCount * sizeof(unsigned int));
        }
        else
        {
            combinedIndexBuffer->SetSize(indexCount, false, false);
            indexData.Resize(indexCount * sizeof(unsigned short));
        }

        // Construct the index buffer
        unsigned currentIndexOffset = 0;
        const unsigned indexSize = indexBuffer_->GetIndexSize();
        for (unsigned objIndex = 0; objIndex < transforms.Size(); ++objIndex, currentIndexOffset += GetVertexCount())
        {
            // Macro gets index at a position, the offsets the value for the object index, and subtracts vertex start (to offset when a VBO is shared with multiple IBOs)
            #define INDEX_ASSIGNMENT(idxType, writeMethod) indexData. writeMethod (*(((idxType *)srcIndexData) + index) + currentIndexOffset - GetVertexStart());
            if (largeIndices)
            {
                if (indexBuffer_->GetIndexSize() == sizeof(unsigned short))
                {
                    for (unsigned index = GetIndexStart(); index < GetIndexStart() + GetIndexCount(); ++index)
                        INDEX_ASSIGNMENT(unsigned short, WriteUInt);
                }
                else
                {
                    for (unsigned index = GetIndexStart(); index < GetIndexStart() + GetIndexCount(); ++index)
                        INDEX_ASSIGNMENT(unsigned, WriteUInt);
                }
            }
            else
            {
                if (indexBuffer_->GetIndexSize() == sizeof(unsigned short))
                {
                    for (unsigned index = 0; index < GetIndexCount(); ++index)
                        INDEX_ASSIGNMENT(unsigned short, WriteUShort);
                }
                else
                {
                    for (unsigned index = GetIndexStart(); index < GetIndexStart() + GetIndexCount(); ++index)
                        INDEX_ASSIGNMENT(unsigned, WriteUShort);
                }
            }
            #undef INDEX_ASSIGNMENT
        }

        combinedIndexBuffer->SetData(indexData.GetModifiableData());
        combinedGeometry->SetIndexBuffer(combinedIndexBuffer);
    }

    // Set our draw range now that index buffer is finished
    combinedGeometry->SetDrawRange(TRIANGLE_LIST, 0, indexCount, false);
    combinedGeometry->indexStart_ = 0;
    combinedGeometry->indexCount_ = indexCount;
    combinedGeometry->vertexStart_ = 0;
    combinedGeometry->vertexCount_ = GetVertexCount() * transforms.Size();
    return combinedGeometry;
}

Geometry* Geometry::CreateConcatenatedList(unsigned numberOfInstances) const
{
    PODVector<Matrix3x4> transforms;
    Matrix3x4 transMat = Matrix3x4::IDENTITY;
    for (unsigned i = 0; i < numberOfInstances; ++i)
        transforms.Push(transMat);
    return CreatePretransformedList(transforms, false);
}

}
