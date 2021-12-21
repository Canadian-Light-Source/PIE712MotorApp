#ifdef _DEBUG
  #undef _DEBUG
  #include <Python.h>
  #define _DEBUG
#else
  #include <Python.h>
#endif

#include <unistd.h>
#include <stdio.h>
#include <errno.h>


#define MAX_STRING_LEN 100000

bool ready = false;
char *t_result;
char cwd[1024];
char curr_path[1024];
char pypath[1024];
char py_cmd[1024];
char *load_param_file(const char* fname)
{
	char cmd[1024];
	PyObject* myModuleString;
	PyObject* myModule;
	PyObject* myFunction;
	PyObject* args;
	PyObject* myResult;
	
	static char *result;
	
  if(! ready){
  	printf("load_param_file: first call, allocating memory\n");
  	t_result = (char *)malloc(sizeof(char) * MAX_STRING_LEN);
  	ready = true;
  	
   	if (getcwd(cwd, sizeof(cwd)) != NULL){
   		sprintf(curr_path, "%s\\python", cwd);
   	} else {
   		printf("There is a problem getting the current working directory\n");
   		sprintf(curr_path, ".\\python");
   	}
   
  }
  printf("the fname passed into load_param_file: is [%s]\n", fname);
  Py_Initialize();
  PyRun_SimpleString("import sys");
  //PyRun_SimpleString("print sys.path");
  sprintf(py_cmd,"pypath = r'%s'", curr_path);
  PyRun_SimpleString(py_cmd);
  
  //sprintf(py_cmd,"pypath = pypath.replace('\','\\'make )", curr_path);
  //PyRun_SimpleString(py_cmd);
  
  sprintf(pypath, "sys.path.insert(0,pypath)");  
	PyRun_SimpleString(pypath);
	
	myModuleString = PyUnicode_FromString((char*)"e712_load_parm_file");
	myModule = PyImport_Import(myModuleString);
	if (myModule != NULL) {
		// Do something useful here
	} else {
	   PyErr_Print();  // print traceback
	   printf("Failed to load e712_load_parm_file\n");
	   return NULL;
	}
	myFunction = PyObject_GetAttrString(myModule,(char*)"load_param_file");
	args = PyTuple_Pack(1,PyUnicode_FromString((char*)fname));

 	if (myFunction && PyCallable_Check(myFunction)) 
  {
		myResult = PyObject_CallObject(myFunction, args);
		Py_DECREF(myFunction);
		Py_DECREF(args);
		if (myResult != NULL) {
			// Do something useful here
			result = (char *)PyUnicode_AsUTF8(myResult);
			Py_DECREF(myResult);
			if(result == NULL)
			{
				PyErr_Print();
			} 
			else {
					//printf("the length of the string is %d bytes\n", strlen(result));
					sprintf(t_result, "%s", result);
					//t_result = result;
			}
		} else {
				PyErr_Print();
			  result = NULL;
		}
	} else {
		PyErr_Print();
		result = NULL;
	}
		
		Py_DECREF(myModuleString);
		Py_DECREF(myModule);
		
		Py_Finalize();	
		if(result == NULL){
			return(NULL);
		} else {
			return(t_result);
		}
}

int exec_python_code(const char* code)
{
	
  if(! ready){
  	Py_Initialize();
  	ready = true;
  }
  
  PyRun_SimpleString(code);
  //Py_Finalize();
  return(0);
}