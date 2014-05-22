#ifndef OBJECT_H
#define OBJECT_H

#define   OBJECT_MAX       30

namespace ar_object
{
  typedef struct
  {
    char name[256];
    int id;
    int visible;
    int collide;
    double marker_coord[4][2];
    double trans[3][4];
    double marker_width;
    double marker_center[2];
  } ObjectData_T;

  ObjectData_T *read_ObjData (char *name, int *objectnum);
}
#endif
