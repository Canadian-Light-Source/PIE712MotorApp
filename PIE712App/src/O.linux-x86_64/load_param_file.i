# 1 "../load_param_file.st"
# 1 "/home/bergr/git_sandbox/cs_apps/Beamlines/SM_Beamline/STXM/ASTXM/AStxm_PIE712App/PIE712App/src/O.linux-x86_64//"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "/usr/include/stdc-predef.h" 1 3 4
# 1 "<command-line>" 2
# 1 "../load_param_file.st"
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



     when (1)
   {
    printf("calling load_parameter_file(%s)\n", parm_filename);



     } state finish


   }





  state finish
  {

   when(delay(1.0))
   {
    printf("Done executing\n");

   } state wait_for_load_start

  }
}

%{
# 96 "../load_param_file.st"
}%
