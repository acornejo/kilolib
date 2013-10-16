#ifndef __RINGBUFFER_H__
#define __RINGBUFFER_H__

#ifndef RB_counter
#define RB_counter unsigned int
#endif

#define NEEDSPACE
#ifndef NEEDSPACE

#define RB_create(NAME, T, SIZE) \
    struct {\
        RB_counter head;\
        RB_counter tail;\
        T elems[SIZE];\
    } NAME;\
    static const RB_counter NAME##_rbsize = SIZE

#define RB_init(o) {\
    o.head = 0;\
    o.tail = 0;\
}

#define RB_clear(o) {\
    o.head = 0;\
    o.tail = 0;\
}

#define RB_capacity(o) o##_rbsize

#define RB_size(o)  (o.head <= o.tail ? o.tail-o.head : o##_rbsize-(o.head-o.tail))

#define RB_empty(o) (o.head == o.tail)

#define RB_full(o)  ((o.head+1)%o##_rbsize == o.tail)

#define RB_front(o) o.elems[o.head]

#define RB_back(o) o.elems[o.tail]

#define RB_popfront(o) o.head = (o.head+1)%o##_rbsize;

#define RB_pushback(o) {\
    o.tail = (o.tail+1)%o##_rbsize;\
    if (RB_empty(o))\
        o.head = (o.head+1)%o##_rbsize;\
}

#else

#define RB_create(NAME, T, SIZE)\
    struct {\
        RB_counter head;\
        RB_counter count;\
        T elems[SIZE];\
    } NAME;\
    static const RB_counter NAME##_rbsize = SIZE

#define RB_init(o) {\
    o.head = 0;\
    o.count = 0;\
}

#define RB_clear(o) {\
    o.head = 0;\
    o.count = 0;\
}

#define RB_capacity(o) o##_rbsize

#define RB_size(o) o.count

#define RB_empty(o) (o.count == 0)

#define RB_full(o)  (o.count == o##_rbsize)

#define RB_front(o) o.elems[o.head]

#define RB_back(o)  o.elems[(o.head+o.count)%o##_rbsize]

#define RB_popfront(o) {\
    o.head = (o.head+1)%o##_rbsize;\
    o.count--;\
}

#define RB_pushback(o) {\
    if (RB_full(o))\
        o.head = (o.head+1)%o##_rbsize;\
    else\
        o.count = o.count + 1;\
}

#define RB_idx(o,i) o.elems[(o.head+i)%o##_rbsize]

#define RB_begin(o) o.head

#define RB_next(o,i) i = ((i+1)%o##_rbsize)

#define RB_end(o)  ((o.head+o.count)%o##_rbsize)

#if 0
#define RB_begin(o) &o.elems[o.head]

#define RB_next(o,i) {\
    i++;\
    if (i == &o.elems[o##_rbsize])\
        i = &o.elems[0];\
}

#define RB_end(o)  &o.elems[(o.head+o.count)%o##_rbsize]
#endif

#endif

#endif//__RINGBUFFER_H__


