function afprintf(st)

  global st__;
  fprintf(st);
  st__ = [st__ st];
