#pragma once

/**
 * @brief Enum class representing the type of command.
 */
enum class CommandType{
    FORWARD,   ///< Command to move forward
    BACKWARD,  ///< Command to move backward
    LEFT,      ///< Command to turn left
    RIGHT,     ///< Command to turn right
    NONE       ///< No command
};

/**
 * @brief Enum class representing the state of a command.
 */
enum class CommandState {
    IDLE,       ///< Command is idle, not yet executed
    EXECUTING,  ///< Command is currently being executed
    COMPLETED   ///< Command has been completed
};

/**
 * @brief Struct representing a command with type, distance, angle, and state.
 */
typedef struct {
    CommandType type;    ///< Type of the command (forward, backward, etc.)
    int distance;        ///< Distance in cm for forward/backward commands
    int angle;           ///< Angle in degrees for left/right commands
    CommandState state;  ///< Current state of the command (idle, executing, completed)
} Command;

/**
 * @brief Class for managing a queue of commands.
 */
class CommandQueue{
    private:     
        static const size_t MaxCommands = 100;  ///< Maximum number of commands in the queue
        Command commands[MaxCommands];          ///< Array to store commands
        size_t headPointer;                     ///< Index for the front of the queue
        size_t tailPointer;                     ///< Index for the back of the queue
        size_t commandCount;                    ///< Number of commands currently in the queue
    
    public:
         /**
         * @brief Constructor initializes the queue pointers and command count.
         */
        CommandQueue():headPointer(0),tailPointer(0),commandCount(0)
        {         
        }
        /**
         * @brief Checks if the command queue is full.
         * @return True if the queue is full, false otherwise.
         */
        bool isFull(){
            return commandCount == MaxCommands;
        }
        /**
         * @brief Checks if the command queue is empty.
         * @return True if the queue is empty, false otherwise.
         */
        bool isEmpty(){
            return commandCount == 0;
        }
        /**
         * @brief Adds a new command to the queue.
         * @param command The command to be added to the queue.
         */
        void push(const Command& command){
            if(!isFull()){
                commands[tailPointer] = command;
                tailPointer = (tailPointer + 1) % MaxCommands;
                commandCount++;
            }
        }
        /**
         * @brief Removes and returns the command at the front of the queue.
         * @return The command at the front of the queue. If the queue is empty, returns a default command with type NONE.
         */
        Command pop(){
            Command command;
            if(!isEmpty()){
                command = commands[headPointer];
                headPointer = (headPointer+1) % MaxCommands;
                commandCount--;
            }else{
                command.type = CommandType::NONE; 
                command.distance = -1;
                command.angle = -1;
                command.state = CommandState::COMPLETED;
            }
            return command;
        }
        /**
         * @brief Gets the current size of the command queue.
         * @return The number of commands currently in the queue.
         */
        size_t size(){
            return commandCount;
        }
        /**
         * @brief Clears all commands in the queue and resets the queue pointers.
         */

        void Clear(){
        for (size_t i = 0; i < MaxCommands; ++i) {
            commands[i].type = CommandType::NONE; 
            commands[i].distance = -1;
            commands[i].angle = -1;
            commands[i].state = CommandState::COMPLETED;
        }
        headPointer = 0;
        tailPointer = 0;
        commandCount = 0;
            
        }
};
