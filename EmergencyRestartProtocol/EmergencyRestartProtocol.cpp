extern "C"
{
#include <sys/file.h> //flock()
#include <unistd.h>   // fork() etc.
}

#include <chrono>
#include <ext/stdio_filebuf.h>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#define NODE_DUMP_PATH "../../node.dump"

int main( int argc, char** argv )
{
	std::fstream dumpFile;
	std::this_thread::sleep_for( std::chrono::milliseconds( 2000 ) );

	while( true )
	{
		std::vector< std::string > crashedNodes;

		dumpFile.open( NODE_DUMP_PATH, std::ios::in );

		int fd = static_cast< __gnu_cxx::stdio_filebuf< char >* const >( dumpFile.rdbuf() )->fd();
		while( flock( fd, LOCK_EX ) != 0 )
		{
			std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
		}

		if( dumpFile.is_open() )
		{
			std::string line;
			while( std::getline( dumpFile, line ) )
			{
				crashedNodes.push_back( line );
			}
		}

		for( auto i = 0u; i < crashedNodes.size(); ++i )
		{
			const auto& in = crashedNodes.at( i );
			const int pid  = fork();
			if( pid == 0 )
			{
				dumpFile.close();
				flock( fd, LOCK_UN );
				std::cout << "Reviving Node " << in << std::endl;
				std::string command = "./../" + in + "/" + in;
				system( command.c_str() );
				goto finishProtocol;
			}
		}
		dumpFile.close();
		dumpFile.open( NODE_DUMP_PATH, std::ios::out | std::ios::trunc );
		dumpFile.close();

		flock( fd, LOCK_UN );
		std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
	}
finishProtocol:
	exit( 0 );
}