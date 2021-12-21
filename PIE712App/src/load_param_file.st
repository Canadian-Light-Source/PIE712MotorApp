program load_param_file

char parm_filename[40];

%%extern void load_parameter_file(char *fname);
ss _load_file
{
    state init_load
    {
	  	entry	
			{
				if (macValueGet("param_file") != 0)
				{
						sprintf(parm_filename, "%s", macValueGet("param_file"));
				} else{
						printf("load_param_file: NO PARAM FILE SPECIFIED!!!");
						exit(1);
				}
				printf("param_file = %s\n", parm_filename);

			}
			
			when (delay(1.0))
			{
				
			} state wait_for_load_start	
		}	
			
			
		state wait_for_load_start
		{	
			/*
		 	*  start calibration procedure if Detz is calibrated and at a safe distance
		 	*/
	    when (1)
			{
				printf("calling load_parameter_file(%s)\n", parm_filename);
				/*%%load_parameter_file(parm_filename);*/
				
				
	    } state finish
	    
	    
	  }
		
		
		/*
		*  we are paranoid so check it again!
		*/
		state finish
		{
			
			when(delay(1.0))
			{
				printf("Done executing\n");
				
			} state wait_for_load_start
			
		}
}

%{
/*
#if __linux
#include <sys/syscall.h>

#elif defined(_WIN32) || defined(_WIN64)

#include <windows.h>

#endif

   
void load_parameter_file(char *fname)
{
	
	char cmd[256];
	sprintf(cmd, "python e712_load_parm_file.py %s", fname);
	system(cmd);

}

#include <Python.h>

void exec_pycode(const char* code)
{
	char cmd[1024];
	
	sprintf(cmd, "from bcm.tests.e712_load_parm_file import load_it \nload_it()");
	
  Py_Initialize();
  PyRun_SimpleString(cmd);
  Py_Finalize();
}
*/
}%
