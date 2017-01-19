#!/bin/bash
echo "If prompted for password, just hit enter (it's blank)"
scp lib/native/lib/*.so admin@roborio-1736-frc.local:/usr/local/frc/lib/
pause