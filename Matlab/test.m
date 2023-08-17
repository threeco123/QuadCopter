delete(instrfindall)
s = serial('COM10','BaudRate',115200);
fopen(s);
%%
flushinput(s)
fwrite(s,'a','sync')
%%
flushinput(s)
fwrite(s,'b','sync')
%%
flushinput(s)
fwrite(s,'a 123 46');
fscanf(s,'%f');