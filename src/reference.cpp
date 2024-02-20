#include <nori/reference.h>

NORI_NAMESPACE_BEGIN

void Reference::addChild(NoriObject *obj){
    switch (obj->getClassType()) {
        case EMesh:
            if (m_shape)
                throw NoriException(
                    "Alreay have refered shape");
            
            m_shape = static_cast<Shape *>(obj);

            break;
        default:
            throw NoriException("Shape::addChild(<%s>) is not supported!",
                                classTypeName(obj->getClassType()));
    }
}
NORI_REGISTER_CLASS(Reference, "ref");
NORI_NAMESPACE_END