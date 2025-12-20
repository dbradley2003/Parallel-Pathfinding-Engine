
#include "Maze.h"

#include <random>
#include <iostream>
#include <fstream>
#include <string>
#include "Direction.h"
#include <cstring>
#include <cassert>

enum class InternalBit
{
	EAST_BIT = 1,
	SOUTH_BIT = 2,
};

struct FileHeader
{
	int width;
	int height;
	int solvable;
};

Maze::Maze()
{
	this->height = 0;
	this->width = 0;
	this->poMazeData = nullptr;
}

Maze::~Maze()
{
	delete this->poMazeData;
}

Maze::Maze(int _width, int _height)
{
	this->height = _height;
	this->width = _width;
	this->poMazeData = nullptr;
	this->solvable = 0;
}

bool Maze::canMove(Position pos, Direction dir)
{
	bool status = false;

	switch (dir)
	{
	case Direction::North:
		if (pos.row == 0)
		{
			status = false;
		}
		else
		{
			status = ((getCell(pos.move(Direction::North)) & (int)InternalBit::SOUTH_BIT) == 0);
		}
		break;

	case Direction::South:
		status = ((getCell(pos) & (int)InternalBit::SOUTH_BIT) == 0);
		break;

	case Direction::East:
		status = ((getCell(pos) & (int)InternalBit::EAST_BIT) == 0);
		break;

	case Direction::West:
		if (pos.col == 0)
		{
			status = false;
		}
		else
		{
			status = ((getCell(pos.move(Direction::West)) & (int)InternalBit::EAST_BIT) == 0);
		}
		break;

	case Direction::Uninitialized:
	default:
		assert(false);
		break;
	}

	return status;

}

ListDirection Maze::getMoves(Position pos)
{
	ListDirection result;

	if (canMove(pos, Direction::South))
	{
		result.south = Direction::South;
	}
	if (canMove(pos, Direction::East))
	{
		result.east = Direction::East;
	}
	if (canMove(pos, Direction::West))
	{
		result.west = Direction::West;
	}
	if (canMove(pos, Direction::North))
	{
		result.north = Direction::North;
	}

	return result;
}

Position Maze::getStart()
{
	Position tmp = Position(0, this->width / 2);
	return tmp;
}

Position Maze::getEnd()
{
	Position tmp = Position(this->height - 1, this->width / 2);
	return tmp;
}

void Maze::setEast(Position pos)
{
	unsigned int newVal = getCell(pos) | (unsigned int)InternalBit::EAST_BIT;
	setCell(pos, newVal);
}

void Maze::setSouth(Position pos)
{
	unsigned int newVal = getCell(pos) | (unsigned int)InternalBit::SOUTH_BIT;
	setCell(pos, newVal);
}

unsigned int Maze::getCell(Position pos)
{
	unsigned int val = poMazeData[pos.row * this->width + pos.col];
	return val;
}

void Maze::setCell(Position pos, unsigned int newValue)
{
	this->poMazeData[pos.row * this->width + pos.col] = newValue;
}


std::streamsize getFileSize(std::ifstream& file)
{
	std::streamsize size = file.tellg();

	file.seekg(0, std::ios::beg);

	return size;
}


std::vector<char> getFileBuffer(const std::string& fileName)
{
	std::ifstream inFile(fileName, std::ios::binary | std::ios::ate);

	if (!inFile.is_open())
	{
		throw std::runtime_error("Error: Could not open file " + fileName);
	}

	std::streamsize fileSize = getFileSize(inFile);

	std::vector<char> buffer(fileSize);
	inFile.read(buffer.data(), fileSize);
	inFile.close();

	return buffer;
}


void Maze::Load(const std::string& fileName)
{
	std::vector<char> buffer = getFileBuffer(fileName);
	char* pBuff = buffer.data();

	FileHeader* pHdr;
	pHdr = (FileHeader*)pBuff;

	this->width = pHdr->width;
	this->height = pHdr->height;
	this->solvable = pHdr->solvable;

	pBuff += sizeof(FileHeader);

	int* pIntData = (int*)pBuff;

	unsigned totalSize = (unsigned int)(this->width * this->height);

	this->poMazeData = new std::atomic<unsigned int>[totalSize];
	memset(this->poMazeData, 0x0, (this->width * this->height) * sizeof(unsigned int));
	Position pos = Position(0, 0);
	while (pos.row < height)
	{
		pos = Position(pos.row, 0);
		while (pos.col < width)
		{
			int bits = *pIntData++;

			for (int bit = 0; (bit < 16 && pos.col < width); bit++)
			{
				if ((bits & 1) == 1)
				{
					setEast(pos);
				}

				if ((bits & 2) == 2)
				{
					setSouth(pos);
				}

				bits >>= 2;
				pos = pos.move(Direction::East);
			}
		}
		pos = pos.move(Direction::South);
	}
}

bool Maze::checkSolution(std::vector<Direction>& soln)
{
	assert(&soln);

	bool results = true;
	Position at = getStart();

	for (auto iter = begin(soln); iter != end(soln); ++iter)
	{
		Direction dir = *iter;


		if (!canMove(at, dir))
		{
			results = false;
			break;
		}

		at = at.move(dir);
	}

	if (!(at == getEnd()))
	{
		results = false;
	}

	if (results)
	{
		std::cout << "    checkSolution(" << (int)soln.size() << " " << "elements):" << " passed\n";

	}
	else
	{
		std::cout << "    checkSolution(" << (int)soln.size() << " " "elements):" << " FAILED!!!\n";


	}

	return results;
}




