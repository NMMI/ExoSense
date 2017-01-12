if exist(strcat(pwd, '/bin'), 'file')
    rmdir('bin', 's');
else
	mkdir(pwd, 'bin');
end

mex -outdir bin src/qbexo.cpp ../../qbAPI/src/qbmove_communications.cpp ../../qbAPI/src/exosense_communications.cpp -Iinclude/