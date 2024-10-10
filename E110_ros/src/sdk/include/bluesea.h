#ifndef __BLUESEA_H__
#define __BLUESEA_H__




#include "LidarDataProcess.h"
#include "error.h"
#define SDKVERSION "1.0" // SDK版本号


class BlueSeaLidarSDK
{
public:
	static BlueSeaLidarSDK *getInstance();
	static void deleteInstance();
    
	/************************************************
	* @functionName:  getLidar
	* @date:          2024-09-25
	* @description:   CN:获取当前雷达的信息  EN:get current lidar info
	* @Parameter:
	*				  1.ID[int,IN]			CN:雷达ID			 EN:lidar ID
	*
	* @return:        RunConfig  struct
	* @others:        Null
	*************************************************/
	RunConfig * getLidar(int ID);
	/************************************************
	* @functionName:  addLidarByPath
	* @date:          2024-09-25
	* @description:   CN:通过配置文件的路径添加ID  EN:add lidar by path
	* @Parameter:
	*				  1.cfg_file_name[std::string,IN]			CN:配置文件路径			 EN:config file path
	*
	* @return:        LIDAR ID
	* @others:        Null
	*************************************************/
	int addLidarByPath(std::string cfg_file_name);
	/************************************************
	* @functionName:  addLidarByPath
	* @date:          2024-09-25
	* @description:   CN:通过配置文件的路径添加ID  EN:add lidar by path
	* @Parameter:
	*				  1.cfg_file_name[std::string,IN]			CN:配置文件路径			 EN:config file path
	*
	* @return:        LIDAR ID
	* @others:        Null
	*************************************************/
	int addLidar(std::string com,int baudrate);
	/************************************************
	* @functionName:  delLidarByID
	* @date:          2024-09-25
	* @description:   CN:通过addLidarByPath返回的ID删除指定雷达  EN:Delete the specified lidar by the ID returned by addLidarByPath
	* @Parameter:
	*				  1.ID[int,IN]			CN:雷达ID			 EN:Lidar ID
	*
	* @return:        true/false
	* @others:        Null
	*************************************************/
	bool delLidarByID(int ID);
	/************************************************
	* @functionName:  setCallBackPtr
	* @date:          2024-09-25
	* @description:   CN:根据ID设置雷达的回调打印函数  EN:Setting up a callback print function for the radar based on an ID
	* @Parameter:
	*				  1.ID[int,IN]			CN:雷达ID			 EN:Lidar ID
	*				  1.ptr[printfMsg,IN]	CN:回调打印函数		EN:Callback print function
	*
	* @return:        NUll
	* @others:        Null
	*************************************************/
	void setCallBackPtr(int ID,printfMsg ptr);
	/************************************************
	* @functionName:  connect
	* @date:          2024-09-25
	* @description:   CN:连接雷达  EN:connect lidar
	* @Parameter:
	*				  1.ID[int,IN]			CN:雷达ID			 EN:Lidar ID
	*
	* @return:        true/false
	* @others:        Null
	*************************************************/
	bool connect(int ID);
	/************************************************
	* @functionName:  disconnect
	* @date:          2024-09-25
	* @description:   CN:断开雷达  EN:disconnect lidar
	* @Parameter:
	*				  1.ID[int,IN]			CN:雷达ID			 EN:Lidar ID
	*
	* @return:        Null
	* @others:        Null
	*************************************************/
	void disconnect(int ID);

	/************************************************
	 * @functionName:  setWork
	 * @date:          2024-09-25
	 * @description:   CN:设置雷达开启或者暂停  EN:Set the lidar on or off
	 * @Parameter:	  
	 *					1.ID[int,IN]			CN:雷达ID			 EN:Lidar ID
	 *					2.isrun	[bool,IN]   CN:是否运行	EN: whether  or not run 
	 * @return:			true/false
	 * @others:        Null
	 *************************************************/
	bool setWork(int ID, bool isrun);


	int GetAllFans(int ID);
	
	std::string getUUID(int ID);
	std::string getModel(int ID);
	std::string hardwareVersion(int ID);
	std::string softwareVersion(int ID);

protected:
	

private:
	static BlueSeaLidarSDK *m_sdk;
	BlueSeaLidarSDK();
	~BlueSeaLidarSDK();

	int m_idx;
	std::vector<RunConfig*> m_lidars;
	

};
void* lidar_service(void* param);
//默认的打印函数
void CallBackMsg(int msgtype, void *param,int length);

#endif