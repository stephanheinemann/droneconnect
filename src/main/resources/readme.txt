====
    Copyright (c) 2016, Stephan Heinemann, Charlie Magnuson, Manuel Rosa (UVic Center for Aerospace Research)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification,
    are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
    may be used to endorse or promote products derived from this software without
    specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
====

Installation Instructions

The Maven gRPC plugins should automatically generate both the Java and Python
client and server files containing the gRPC stubs (RPCs) and data types
(messages) from the protocol buffer definition file under the "src/main/proto"
directory. Both grpc and grpc-java should be installed on the build environment.
See github.com/grpc for more information.

Whenever the protocol buffer definition file changes, the client and server side
stubs and data types have to be re-generated. These files can also be manually
generated from the command line using the provided shell scripts

"src/main/resources/generate-client.sh" and
"src/main/resources/generate-server.sh", respectively.

For more information on gRPC visit grpc.io/docs.

Finally, the provided Python server has to be deployed on the target system by
adding the following line to "/etc/rc.local" where <path> has to be substituted
accordingly:

python <path>/droneconnect_server.py &>> <path>/droneconnect_server_log.txt &
