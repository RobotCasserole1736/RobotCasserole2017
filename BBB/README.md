# Casserole BBB Vision Processing

Many many many thanks to FRC2481 for their simple instructions on getting this started.

Their awesome work can be found [here](https://github.com/Frc2481/paul-bunyan).

For info on our setup, see our [wiki page](https://github.com/RobotCasserole1736/RobotCasserole2017/wiki/Vision-Target-Identification-System).

## id_rsa.pub

To copy the id_rsa.pub public key to the BBB (a one time process for a new device), you need to run the following from a UNIX based command prompt, or MinGW works if you're using Windows (alternatively you can copy the file and set permissions by any other means):

~~~~
cat id_rsa.pub | ssh user@hostname 'cat >>
	~/.ssh/authorized_keys &&
	chmod 700 ~/.ssh/ &&
	chmod 600 ~/.ssh/authorized_keys'
~~~~