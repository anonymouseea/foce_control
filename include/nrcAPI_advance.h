#ifndef API_NRCAPI_ADVANCE_H_
#define API_NRCAPI_ADVANCE_H_

#include "nrcAPI.h"
#include "json/json.h"

/*************JSON文件操作*************/
/**
 *	@brief 以json格式读取模型参数文件
 */
int NRC_ReadModelFile(Json::Value& root);

/**
 *	@brief 以json格式写入模型参数文件，请配合NRC_ReadModelFile函数使用
 */
int NRC_ModifyModelFile(Json::Value& root);
/**
 *	@brief 读取特定路径下的Json文件,读取成功返回文件大小，读取失败返回-1
 */
int NRC_ReadJsonFileToJson(const char *filepath, Json::Value& root);

/**
 *	@brief 保存特定路径下的Json文件,保存成功返回文件大小，保存失败返回-1
 */
int NRC_SaveJsonToJsonFile(const char *filepath, Json::Value& root);

/**************进阶接口**************/
/**
 *	@brief 设置目标位置
 *	@note 此接口会直接给伺服下发目标位置，使用时请确保位置正确，否则有飞车风险
 *	@param　axis 轴号
 *	@param　targetPosition 目标位置
 */
void NRC_SetTargetAngle(int axis, double targetPosition);

/**************作业文件专用**************/
/**
 * @brief 供自定义作业文件指令调用的MOVJ指令，不可直接调用
 * @param robotNum 机器人编号[1-4]
 * @param pos　机器人运动目标位置，详见 NRC_Position
 * @param vel 机器人的运行速度，为机器人最大速度的百分比，参数范围：0 < vel <= 100
 * @param acc 机器人运行加速度，为机器人各关节最大加速度的百分比，参数范围：0 < vel <= 100
 * @param dec 机器人运行减速度，为机器人各关节最大减速度的百分比，参数范围：0 < vel <= 100
 * @param pl 平滑度，将和后面一条移动指令进行平滑过渡，数值越大，越平滑，轨迹偏差也越大，参数范围：0 <= pl <= 5
 */
void NRC_Jobrun_MoveDirect(int robotNum, const NRC_Position & pos, double vel,  double acc, double dec, int pl);

/**
 * @brief 供自定义作业文件指令调用的MOVL指令，不可直接调用
 * @param robotNum 机器人编号[1-4]
 * @param pos　机器人运动目标位置，详见 NRC_Position
 * @param vel 机器人的运行速度，为机器人末端位置点绝对运行速度，单位为 毫米每秒（mm/s），参数范围：vel > 1
 * @param acc 机器人运行加速度，为机器人各关节最大加速度的百分比，参数范围：0 < vel <= 100
 * @param dec 机器人运行减速度，为机器人各关节最大减速度的百分比，参数范围：0 < vel <= 100
 * @param pl 平滑度，将和后面一条移动指令进行平滑过渡，数值越大，越平滑，轨迹偏差也越大，参数范围：0 <= pl <= 5
 */
void NRC_Jobrun_MoveLinear(int robotNum, const NRC_Position & pos, double vel, double acc, double dec, int pl);

/**
 * @brief 供自定义作业文件指令调用的MOVC指令，不可直接调用
 * @param robotNum 机器人编号[1-4]
 * @param mid_pos,end_pos　机器人运动目标位置，详见 NRC_Position
 * @param vel 机器人的运行速度，为机器人末端位置点绝对运行速度，单位为 毫米每秒（mm/s），参数范围：vel > 1
 * @param acc 机器人运行加速度，为机器人各关节最大加速度的百分比，参数范围：0 < vel <= 100
 * @param dec 机器人运行减速度，为机器人各关节最大减速度的百分比，参数范围：0 < vel <= 100
 * @param pl 平滑度，将和后面一条移动指令进行平滑过渡，数值越大，越平滑，轨迹偏差也越大，参数范围：0 <= pl <= 5
 * @note 注意，MOVC不可作为第一条指令运行，且第一次调用时前面必须要有MOVJ或者MOVL指令
 */
void NRC_Jobrun_MoveC(int robotNum, const NRC_Position& mid_pos,const NRC_Position& end_pos, double vel, double acc, double dec, int pl);

/**
 * @brief 供自定义作业文件指令调用的MOVS指令，不可直接调用
 * @param robotNum 机器人编号[1-4]
 * @param pos 机器人运动目标位置容器，大小不可小于4(即pos.size() >= 4)，点位设置详见 NRC_Position
 * @param vel 机器人的运行速度，为机器人末端位置点绝对运行速度，单位为 毫米每秒（mm/s），参数范围：vel > 1
 * @param acc 机器人运行加速度，为机器人各关节最大加速度的百分比，参数范围：0 < vel <= 100
 * @param dec 机器人运行减速度，为机器人各关节最大减速度的百分比，参数范围：0 < vel <= 100
 * @param pl 平滑度，将和后面一条移动指令进行平滑过渡，数值越大，越平滑，轨迹偏差也越大，参数范围：0 <= pl <= 5
 * @note 注意，MOVS不可作为第一条指令运行，且第一次调用时前面必须要有MOVJ或者MOVL指令
 */
void NRC_Jobrun_MoveS(int robotNum, int pointNum, const std::vector< NRC_Position>& pos, double vel, double acc, double dec, int pl);

/**
 * @brief 供自定义作业文件指令调用的IMOV指令，不可直接调用
 * @param robotNum 机器人编号[1-4]
 * @param pos 机器人运动目标位置容器，大小不可小于4(即pos.size() >= 4)，点位设置详见 NRC_Position
 * @param vel 机器人的运行速度，为机器人末端位置点绝对运行速度，单位为 毫米每秒（mm/s），参数范围：vel > 1
 * @param acc 机器人运行加速度，为机器人各关节最大加速度的百分比，参数范围：0 < vel <= 100
 * @param dec 机器人运行减速度，为机器人各关节最大减速度的百分比，参数范围：0 < vel <= 100
 * @param pl 平滑度，将和后面一条移动指令进行平滑过渡，数值越大，越平滑，轨迹偏差也越大，参数范围：0 <= pl <= 5
 * @param tm 提前执行时间，范围：[0,999999],可不填，默认为0
 * @note 注意，IMOV不可作为第一条指令运行，在其之前必须要有运动类指令
 */
int NRC_Jobrun_IMOV(int robotNum, const NRC_Position& offset, double vel, double acc, double dec, int pl, int tm = 0);

/**
 * @brief 供自定义作业文件指令调用的MOVJEXT指令，不可直接调用
 * @param robotNum 机器人编号[1-4]
 * @param pos　机器人运动目标位置，详见 NRC_Position
 * @param syncPos　机器人运动目标位置，详见 NRC_SyncPosition
 * @param vel 机器人的运行速度，为机器人最大速度的百分比，参数范围：0 < vel <= 100
 * @param acc 机器人运行加速度，为机器人各关节最大加速度的百分比，参数范围：0 < vel <= 100
 * @param dec 机器人运行减速度，为机器人各关节最大减速度的百分比，参数范围：0 < vel <= 100
 * @param pl 平滑度，将和后面一条移动指令进行平滑过渡，数值越大，越平滑，轨迹偏差也越大，参数范围：0 <= pl <= 5
 */
void NRC_Jobrun_MoveDirectSync(int robotNum, const NRC_Position & pos, const NRC_SyncPosition & syncPos, double vel, double acc, double dec, int pl);

/**
 * @brief 供自定义作业文件指令调用的MOVLEXT指令，不可直接调用
 * @param robotNum 机器人编号[1-4]
 * @param pos　机器人运动目标位置，详见 NRC_Position
 * @param syncPos　机器人运动目标位置，详见 NRC_SyncPosition
 * @param vel 机器人的运行速度，为机器人末端位置点绝对运行速度，单位为 毫米每秒（mm/s），参数范围：vel > 1
 * @param acc 机器人运行加速度，为机器人各关节最大加速度的百分比，参数范围：0 < vel <= 100
 * @param dec 机器人运行减速度，为机器人各关节最大减速度的百分比，参数范围：0 < vel <= 100
 * @param pl 平滑度，将和后面一条移动指令进行平滑过渡，数值越大，越平滑，轨迹偏差也越大，参数范围：0 <= pl <= 5
 * @param sync 是否协同
 */
void NRC_Jobrun_MoveLinearSync(int robotNum, const NRC_Position & pos, const NRC_SyncPosition & syncPos, double vel, double acc, double dec, int pl, int sync);

/**
 * @brief 退出作业文件,通常用于外部结束作业文件，自定义指令中返回false会自动调用此接口
 */
void NRC_JobRun_QuitJob();

/**
 * @brief 清除运行队列中剩余的点位
 */
void NRC_JobRun_ClearRestPos();

/**
 * @brief 设置提前执行时间
 * @param time 提前执行时间，单位ms，设置此参数会使NRC_JobRun_BlockNotMoveInstruction()接口在被阻塞时提前设置的时间被释放
 * 举例说明，当前在执行的运动指令还有３秒才跑完，若设置提前执行时间1000ms，则阻塞会在最后差一秒跑完的地方释放
 */
void NRC_JobRun_SetAheadTime(double time);

/**
 * @brief 阻塞非运动指令，当当前存在机器人运动指令未执行完成时，会阻塞在此接口中，阻止程序向下执行;
 * @brief 在断点恢复时，程序会从停止时程序停留的此接口处继续向下执行
 */
void NRC_JobRun_BlockNotMoveInstruction();

/**
 * @brief 屏蔽运动指令执行完成时自动跳行（注：任何运动类指令在执行完成的时候会执行一次跳行）
 * @brief 当同一条作业文件指令调用多条运动指令时，需要调用此接口屏蔽跳行，并在此指令最后调用NRC_JobRun_SendDone()手动跳行
 */
void NRC_JobRun_SetPauseSendDoneMsg(bool flag);

/**
 * @brief 获取作业文件退出标志位，一般跟在NRC_JobRun_BlockNotMoveInstruction()接口后面使用
 * @brief 当作业文件重新运行或清断点时，上一次的未运行完成的作业文件需要通过此接口进行判断退出，否则新作业文件指令无法正确执行
 */
bool NRC_JobRun_GetInstructionAheadReturn();

/**
 * @brief 跳行接口，调用此指令时，若当前没有正在执行的运动指令，会立刻使作业文件跳行，否则会等待运动指令执行完成后跳行
 * @brief 自定义指令在未调用运动类接口时，必须调用此指令手动跳行
 */
void NRC_JobRun_SendDone();

/**
 * @brief 获取最后一条执行的指令
 * @brief 一般用在运动指令开头来判断上一条指令是否可以衔接平滑
 * @note 只有运动指令之间可以衔接平滑，外部轴点位和非外部轴点位运动之间无法衔接平滑
 */
std::string NRC_JobRun_GetLastCmdType();

/**
 * @brief 获取当前程序已经走到的指令类型
 * @param num　前或后偏移num条指令
 */
std::string NRC_JobRun_GetCurrentRunCmdType(int robotNum, int num = 0);

/**
 * @brief 获取当前程序已经走到的指令序列号
 * @brief　每次作业文件执行时每条指令都会生成唯一的序列号，一般用于区分同类型的不同指令
 * @param num　前或后偏移num条指令
 */
int NRC_JobRun_GetCurrentRunSerialNumber(int robotNum,int num = 0);

/**
 * @brief 获取当前光标所指向的指令类型,指程序已经下发指令，但机器人尚未执行完成的指令
 * @param num　前或后偏移num条指令
 * @brief　由于机器人执行动作往往比运动指令插补慢，所以当前执行的指令一般会快于光标所指向的指令
 */
std::string NRC_JobRun_GetCurrentShowCmdType(int robotNum, int num = 0);

/**
 * @brief 获取当前光标所指向的指令类型序列号
 * @brief　每次作业文件执行时每条指令都会生成唯一的序列号，一般用于区分同类型的不同指令
 * @param num　前或后偏移num条指令
 */
int NRC_JobRun_GetCurrentShowSerialNumber(int robotNum,int num = 0);

/**
 * @brief 获取当前光标所在行的用户自定义指令的id
 * @brief 可用于线程监控等同步控制
 */
int NRC_JobRun_GetCurrentShowUserDefineID();

/**
 *	@brief 设置下一条运动指令的平滑等级，范围[0,5]
 */
void NRC_JobRun_SetPLGrade(int pl);

/**
 *	@brief 获取当前作业文件的局部点位
 *	@param robotNum 机器人编号
 *	@param　posName 点位编号 范围[P0001,P9999]
 *	@param pos 用于存储获取的机器人点位
 */
void NRC_JobRun_GetLocalPosVarValue_P(int robotNum,const std::string &posName,NRC_Position &pos);

/**
 *	@brief 获取当前作业文件的局部外部轴点位
 *	@param robotNum 机器人编号
 *	@param　posName 外部轴点位编号 范围[E0001,E9999]
 *	@param pos 用于存储获取的机器人点位
 *	@param　posSync 用于存储获取的外部轴点位
 */
void NRC_JobRun_GetLocalPosVarValue_E(int robotNum,const std::string &posName,NRC_Position &pos, NRC_SyncPosition &posSync);

/**
 *	@brief 获取当前作业文件的类型
 *	@param robotNum 机器人编号
 *	@return 返回作业文件的类型 0：作业文件 1：后台作业文件 2：全局后台 3：复位点程序
 */
int NRC_JobRun_getJobType(int robotNum);

#endif
