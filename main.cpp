#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>


#include <pthread.h>
#include <system_error>
#include <unistd.h>
#include <memory>
#include <signal.h>
#include <fcntl.h>

#include "./include/USBCam_API/USBCam_API.h"        //videoStreaming Control

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <chrono>
#include <iomanip>
#include <thread>

#include <string>
#include <vector>
#include <map>

#include <zmq.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <random>
#include <cstdlib>
#include "config_reader.h"


using namespace cv;

class PrecisionTimer {
	public:
		using Clock = std::chrono::high_resolution_clock;
		using TimePoint = std::chrono::time_point<Clock>;
		using Milliseconds = std::chrono::milliseconds;
		using Microseconds = std::chrono::microseconds;
		using Nanoseconds = std::chrono::nanoseconds;

		// 开始计时
		void start() {
			start_time = Clock::now();
			running = true;
		}

		// 停止计时
		void stop() {
			end_time = Clock::now();
			running = false;
		}

		// 获取经过的毫秒数（默认精度）
		double elapsed_ms() const {
			return elapsed<Milliseconds>();
		}

		// 获取经过的微秒数
		double elapsed_us() const {
			return elapsed<Microseconds>();
		}

		// 获取经过的纳秒数
		int64_t elapsed_ns() const {
			return elapsed<Nanoseconds>();
		}

		// 格式化输出时间间隔（自动选择最佳单位）
		void print(const std::string& event_name = "") const {
			double ms = elapsed_ms();

			std::cout << std::fixed << std::setprecision(3);
			if (ms >= 1000) {
				std::cout << "[" << event_name << "] " << ms/1000 << " seconds\n";
			} else if (ms >= 1) {
				std::cout << "[" << event_name << "] " << ms << " ms\n";
			} else {
				double us = elapsed_us();
				if (us >= 1) {
					std::cout << "[" << event_name << "] " << us << " μs\n";
				} else {
					std::cout << "[" << event_name << "] " << elapsed_ns() << " ns\n";
				}
			}
		}

	private:
		template<typename Duration>
			auto elapsed() const -> typename Duration::rep {
				auto end = running ? Clock::now() : end_time;
				return std::chrono::duration_cast<Duration>(end - start_time).count();
			}

		TimePoint start_time, end_time;
		bool running = false;
};

static int32_t api_get_thread_policy (pthread_attr_t *attr)
{
	int32_t policy;

	int32_t rs = pthread_attr_getschedpolicy (attr, &policy);

	//assert (rs == 0);

	switch (policy)
	{
		case SCHED_FIFO:
			fprintf (stderr,"policy = SCHED_FIFO\n");
			break;
		case SCHED_RR:
			fprintf (stderr,"policy = SCHED_RR");
			break;
		case SCHED_OTHER:
			fprintf (stderr,"policy = SCHED_OTHER\n");
			break;
		default:
			fprintf (stderr,"policy = UNKNOWN\n");
			break;
	}


	return policy;
}


static void api_set_thread_policy (pthread_attr_t *attr)
{
	//int rs = pthread_attr_setschedpolicy (attr, policy);

	api_get_thread_policy (attr);

	struct sched_param param;

	pthread_attr_setschedpolicy(attr,SCHED_FIFO);
	//设置调度参数
	param.sched_priority = 99;

	pthread_attr_setschedparam(attr,&param);

	pthread_attr_setinheritsched(attr,PTHREAD_EXPLICIT_SCHED);

	pthread_detach(pthread_self());

}


void Set_Thread_attr()
{


	pthread_attr_t attr;
	int32_t     rs;
	rs = pthread_attr_init(&attr);

	if(rs == -1)
	{
		fprintf(stderr,"Error pthread_attr_init \r\n");
	}

	api_set_thread_policy (&attr);


}


void* USBCam_STREAM_DEAL(void*pUSBCam)
{
	Set_Thread_attr();

	Pix_Format Cfg;

	Cfg.u_PixFormat = 0;
	// Cfg.u_Width = 1280;
	// Cfg.u_Height = 800;    

	// Cfg.u_Width = 1024;
	// Cfg.u_Height = 768;

	Cfg.u_Width = 800;
	Cfg.u_Height = 600;
	Cfg.u_Fps = 120;

	TST_USBCam_Video_DEAL_WITH         (pUSBCam,Cfg);

	/************************************************
	  节点配置解除
	 ************************************************/
	TST_USBCam_Video_DEAL_WITH_UNINIT  (pUSBCam);

	TST_USBCam_DELETE_DEVICE_POINT     (pUSBCam);

	return NULL;

}

void *GP3DStram = NULL;

bool EXIT = 1;

void SIG_QUIT(int signal)
{
	if(
			(signal == SIGINT) ||
			(signal == SIGQUIT)
	  )
	{
		EXIT = 0;

		TST_USBCam_EVENT_LoopMode(GP3DStram,0);
	}

}

/**
 * @brief 显示来自void*缓冲区的JPEG图像
 * @param jpeg_buffer 指向JPEG数据的void指针
 * @param buffer_size 缓冲区大小（字节数）
 * @return 成功返回0，失败返回-1
 */
int displayJpegFromVoidBuffer(const void* jpeg_buffer, size_t buffer_size) {
	// 验证输入参数
	if (!jpeg_buffer || buffer_size == 0) {
		std::cerr << "错误：无效的缓冲区指针或大小" << std::endl;
		return -1;
	}

	// 将void*转换为OpenCV需要的无符号字符指针
	const unsigned char* buffer = static_cast<const unsigned char*>(jpeg_buffer);

	try {
		// 使用imdecode解码内存中的JPEG图像
		cv::Mat image = cv::imdecode(
				cv::Mat(1, buffer_size, CV_8UC1, (void*)buffer),  // 创建1xN的单通道矩阵包装缓冲区
				cv::IMREAD_COLOR                                  // 以BGR格式加载彩色图像
				);

		if (image.empty()) {
			std::cerr << "错误：无法解码JPEG数据，可能是损坏的或不支持的格式" << std::endl;
			return -1;
		}

		// 创建可调整大小的窗口
		cv::namedWindow("JPEG Viewer", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);

		// // 显示图像
		cv::imshow("JPEG Viewer", image);

		// 打印图像信息
		std::cout << "成功显示图像: "
			<< image.cols << "x" << image.rows 
			<< " 通道数: " << image.channels() << std::endl;

		// 等待按键
		std::cout << "按任意键退出..." << std::endl;
		auto key = cv::waitKey(1);
		if(key == 27){
			//escpae key
			EXIT = 0;
			// 销毁窗口
			cv::destroyAllWindows();
		}


	} catch (const cv::Exception& e) {
		std::cerr << "OpenCV异常: " << e.what() << std::endl;
		return -1;
	} catch (...) {
		std::cerr << "未知异常发生" << std::endl;
		return -1;
	}

	return 0;
}

/**
 * @brief 将JPEG缓冲区转换为cv::Mat
 * @param jpeg_buffer void*类型的JPEG数据指针
 * @param buffer_size 缓冲区大小（字节数）
 * @return 解码后的cv::Mat (BGR格式)
 * @throws std::runtime_error 当解码失败时抛出异常
 */
cv::Mat jpegBufferToMat(const void* jpeg_buffer, size_t buffer_size) {
	// 参数校验
	if (!jpeg_buffer || buffer_size == 0) {
		throw std::runtime_error("Invalid buffer: null pointer or zero size");
	}

	// 将void*转换为OpenCV需要的格式
	const unsigned char* byte_buffer = static_cast<const unsigned char*>(jpeg_buffer);

	// 使用imdecode解码
	cv::Mat raw_data(1, buffer_size, CV_8UC1, (void*)byte_buffer);
	cv::Mat decoded_image = cv::imdecode(raw_data, cv::IMREAD_COLOR);

	// 验证解码结果
	if (decoded_image.empty()) {
		throw std::runtime_error("Failed to decode JPEG buffer");
	}

	return decoded_image;
}

void displayCenteredText(const std::string& text, 
		cv::Scalar bg_color = cv::Scalar(255, 0, 0),  // 默认蓝色背景 (BGR格式)
		cv::Scalar text_color = cv::Scalar(255, 255, 255),  // 白色文字
		int window_width = 800, 
		int window_height = 600) {

	// 创建指定大小的彩色图像（蓝底）
	cv::Mat image(window_height, window_width, CV_8UC3, bg_color);

	// 设置字体属性
	int font_face = cv::FONT_HERSHEY_DUPLEX;
	double font_scale = 8.0;
	int thickness = 7;

	// 获取文本的包围盒大小
	int baseline = 0;
	cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);

	// 计算文本位置（居中）
	cv::Point text_org(
			(window_width - text_size.width) / 2,
			(window_height + text_size.height) / 2  // OpenCV的y坐标从顶部开始
			);

	// 添加文字阴影效果（增强可读性）
	cv::putText(image, text, text_org + cv::Point(2, 2), 
			font_face, font_scale, cv::Scalar(0, 0, 0),  // 黑色阴影
			thickness, cv::LINE_AA);

	// 绘制主文字
	cv::putText(image, text, text_org, 
			font_face, font_scale, text_color,
			thickness, cv::LINE_AA);

	// 创建可调整大小的窗口
	cv::namedWindow("Display Window", cv::WINDOW_NORMAL);
	cv::resizeWindow("Display Window", window_width, window_height);

	// 显示图像
	cv::imshow("Display Window", image);

	// // 等待按键
	// std::cout << "按任意键退出..." << std::endl;
	// cv::waitKey(0);
	// cv::destroyAllWindows();
}


class MyCommandLineParser {
	private:
		std::map<std::string, std::string> options;
		std::vector<std::string> arguments;

	public:
		void parse(int argc, char* argv[]) {
			for (int i = 1; i < argc; ++i) {
				std::string arg = argv[i];

				if (arg.substr(0, 2) == "--") {
					// 长选项 --option=value 或 --option value
					size_t equal_pos = arg.find('=');
					if (equal_pos != std::string::npos) {
						std::string key = arg.substr(2, equal_pos - 2);
						std::string value = arg.substr(equal_pos + 1);
						options[key] = value;
					} else {
						std::string key = arg.substr(2);
						if (i + 1 < argc && argv[i + 1][0] != '-') {
							options[key] = argv[++i];
						} else {
							options[key] = ""; // 标志选项
						}
					}
				} else if (arg[0] == '-') {
					// 短选项 -o value 或 -abc
					std::string key = arg.substr(1);
					if (i + 1 < argc && argv[i + 1][0] != '-') {
						options[key] = argv[++i];
					} else {
						options[key] = ""; // 标志选项
					}
				} else {
					arguments.push_back(arg);
				}
			}
		}

		bool hasOption(const std::string& option) const {
			return options.find(option) != options.end();
		}

		std::string getOption(const std::string& option, const std::string& defaultValue = "") const {
			auto it = options.find(option);
			if (it != options.end()) {
				return it->second;
			}
			return defaultValue;
		}

		const std::vector<std::string>& getArguments() const {
			return arguments;
		}
};

double distanceBetweenCorners(apriltag_detection_t *det, int i, int j) {
	double dx = det->p[i][0] - det->p[j][0];
	double dy = det->p[i][1] - det->p[j][1];
	return sqrt(dx*dx + dy*dy);
}

int main(int argc, char *argv[])
{

	signal(SIGINT,SIG_QUIT);
	signal(SIGQUIT,SIG_QUIT);

	int32_t fd;

	MyCommandLineParser parser;
	parser.parse(argc, argv);

	if (parser.hasOption("help") || argc == 1) {
		std::cout << "Usage: " << argv[0] << " [OPTIONS]" << std::endl;
		std::cout << "Options:" << std::endl;
		std::cout << "  --help                 Show this help" << std::endl;
		std::cout << "  --version              Show version" << std::endl;
		std::cout << "  --list [COUNT]         List items" << std::endl;
		std::cout << "  --run [NAME]           Run program" << std::endl;
		return 0;
	}

	std::string config_file = "slave_config.conf";
    
    std::cout << "使用配置文件: " << config_file << std::endl;
    
    // 加载配置
    ConfigReader reader(config_file);
    if (!reader.loadConfig()) {
        std::cerr << "加载配置文件失败" << std::endl;
        return 1;
    }
    
    NodeConfig config = reader.getNodeConfig();
    
    if (!reader.validateConfig(config)) {
        std::cerr << "配置验证失败" << std::endl;
        return 1;
    }
    
    if (config.is_master) {
        std::cerr << "错误: 从节点不能使用主节点配置" << std::endl;
        return 1;
    }
    
    std::cout << "从节点配置加载成功:" << std::endl;
    std::cout << "  节点ID: " << config.node_id << std::endl;
    std::cout << "  主节点IP: " << config.master_ip << std::endl;
    std::cout << "  数据端口: " << config.master_data_port << std::endl;
    std::cout << "  命令端口: " << config.master_cmd_port << std::endl;
    std::cout << "  预览窗口: " << config.open_preview << std::endl;
    std::cout << "  标签相对距离阈值: " << config.relative_distance << std::endl;
    	
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_PUB);

	try {
        std::string data_endpoint = "tcp://" + config.master_ip + ":" + 
                                  std::to_string(config.master_data_port);
        socket.connect(data_endpoint);
        std::cout << "节点 " << config.node_id << " 连接到主节点 " << data_endpoint << std::endl;
    } catch (const zmq::error_t& e) {
        std::cerr << "连接失败: " << e.what() << std::endl;
        return 1;
    }

	/************************************************
	  video stream control
	 ************************************************/

	bool listdevice = false;

	if (parser.hasOption("list")) {
		listdevice = true;
	}

	v4l2_dev_sys_data_t *pdevinfo   = NULL;


	int32_t ret =   TST_USBCam_DEVICE_FIND_ID     (&pdevinfo,0xFFFF,0xFFFF);

	if((ret == NULL_RETURN)||(ret == 0))
	{
		fprintf(stdout, "NULL Device\r\n");

		return -1;
	}

	fprintf(stdout,"device find result %d\r\n",ret);

	for(int32_t i = 0;i < ret;i++)
	{
		fprintf(stdout, "VID %04x\r\n", pdevinfo[i].Vid);

		fprintf(stdout, "PID %04x\r\n", pdevinfo[i].Pid);

		fprintf(stdout, "BCD %04x\r\n", pdevinfo[i].Bcd);

		fprintf(stdout, "ST0 %s\r\n",   pdevinfo[i].iManufacturer);

		fprintf(stdout, "ST1 %s\r\n",   pdevinfo[i].iProduct);

		fprintf(stdout, "ST2 %s\r\n",   pdevinfo[i].iSerialNumber);

		fprintf(stdout, "location %s\r\n",   pdevinfo[i].location);
	}

	if(listdevice==true){
		return 0;
	}

	int camera_index = 0;
	if (parser.hasOption("run")) {

		std::string count_str = parser.getOption("run", "0");
		int camera_index = std::stoi(count_str);
		if((camera_index > ret - 1) || (camera_index < 0)){
			std::cout << "Camera " << camera_index << " out off range..." << std::endl;
			return 0;
		}
		std::cout << "Runing " << camera_index << " camera..." << std::endl;
	}
	void* pUSBCam =
		TST_USBCam_CREATE_DEVICE_POINT     (pdevinfo[camera_index]);

	fd = open(pdevinfo[camera_index].Device_Path, O_RDWR | O_NONBLOCK, 0);

	/************************************************
	  在进行开启视频流之前，需打开设备，并将设备描述符传入，如过之前有打开过，将之前打开过的设备描述符传入即可
	 ************************************************/

	int32_t returnVAL = TST_USBCam_Video_DEAL_WITH_INIT     (pUSBCam,fd);

	if(returnVAL != SUCCESS_RETURN)
	{
		fprintf(stderr,"TST_USBCam_Video_DEAL_WITH_INIT Fail ret %d\r\n",returnVAL);

		return -1;
	}

	Pix_Format *ppix_format;

	int32_t pix_format_size = TST_USBCam_Get_Format_List_Size(pUSBCam);

	fprintf(stderr,"pix_format_size:%d\r\n",pix_format_size);

	ppix_format = new Pix_Format[pix_format_size];

	TST_USBCam_Get_Format_List        (pUSBCam,ppix_format);

	for(int32_t i = 0 ; i < pix_format_size ; i++)
	{
		fprintf(stderr," %C%C%C%C %dx%d %d fps\r\n",
				ppix_format[i].u_PixFormat>>0 &0xFF,
				ppix_format[i].u_PixFormat>>8 &0xFF,
				ppix_format[i].u_PixFormat>>16 &0xFF,
				ppix_format[i].u_PixFormat>>24 &0xFF,
				ppix_format[i].u_Width,
				ppix_format[i].u_Height,
				ppix_format[i].u_Fps);
	}


	pthread_t threadId;

	pthread_create(&threadId,   NULL,   USBCam_STREAM_DEAL,  pUSBCam);

	TST_USBCam_Video_STREAM_STATUS(pUSBCam,1);

	/************************************************
	  在视频流开启后才能进行Processing Unit Control
	 ************************************************/
	int32_t pu_val;

	TST_USBCam_PU_Get(pUSBCam,V4L2_CID_EXPOSURE_AUTO,&pu_val);

	fprintf(stderr,"V4L2_EXPOSURE_AUTO val:%d\r\n",pu_val);

	//TST_USBCam_PU_Set(pUSBCam,V4L2_CID_EXPOSURE_AUTO,3); //1:manual 3:AUTO

	if(pu_val == 1)
	{
		TST_USBCam_PU_Get(pUSBCam,V4L2_CID_EXPOSURE_ABSOLUTE,&pu_val);

		fprintf(stderr,"V4L2_CID_EXPOSURE_ABSOLUTE val:%d\r\n",pu_val);

		TST_USBCam_PU_Set(pUSBCam,V4L2_CID_EXPOSURE_ABSOLUTE,pu_val);
	}
	TST_USBCam_PU_Set(pUSBCam,V4L2_CID_EXPOSURE_ABSOLUTE,0);

	// 创建AprilTag检测器
	apriltag_family_t* tf = tag36h11_create();
	apriltag_detector_t* td = apriltag_detector_create();
	apriltag_detector_add_family(td, tf);
	td->quad_decimate = 2.0;  // 降低检测分辨率（加速）
	td->quad_sigma = 0.0;     // 高斯模糊去噪
	td->nthreads = 4;         // 使用4个线程

	PrecisionTimer timer;

	std::string id_to_display = "X";

	PrecisionTimer id_display_timer;

	if(config.open_preview){
		displayCenteredText("X");
	}

	while(EXIT)
	{
		Frame_Buffer_Data*pFrame = TST_USBCam_GET_FRAME_BUFF(pUSBCam,0);


		if(pFrame != NULL)
		{
			// start performace analysize
			timer.start();

			// displayJpegFromVoidBuffer(pFrame->pMem, pFrame->buffer.bytesused);
			// 转换并获取cv::Mat
			cv::Mat result_image = jpegBufferToMat(pFrame->pMem, pFrame->buffer.bytesused);

			cv::Mat frame, gray;

			frame = result_image;

			// 转换为灰度图
			cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

			// 准备AprilTag输入图像
			image_u8_t im = {
				.width = gray.cols,
				.height = gray.rows,
				.stride = gray.cols,
				.buf = gray.data
			};

			// 检测标记
			zarray_t* detections = apriltag_detector_detect(td, &im);

			// 绘制检测结果
			for (int i = 0; i < zarray_size(detections); i++) {
				apriltag_detection_t* det;
				zarray_get(detections, i, &det);

				// 绘制四边形边框
				for (int j = 0; j < 4; j++) {
					cv::Point pt1(det->p[j][0], det->p[j][1]);
					cv::Point pt2(det->p[(j + 1) % 4][0], det->p[(j + 1) % 4][1]);
					cv::line(frame, pt1, pt2, cv::Scalar(0, 255, 0), 2);
				}

				// 显示ID和中心点
				cv::putText(frame, 
						std::to_string(det->id),
						cv::Point(det->c[0], det->c[1]),
						cv::FONT_HERSHEY_SIMPLEX,
						1,
						cv::Scalar(255, 0, 0),
						2);

				// print detected id
				std::cout << "detected id: "<< std::to_string(det->id) << std::endl;

				id_to_display = std::to_string(det->id);
				id_display_timer.start();
				if(config.open_preview){
					displayCenteredText(id_to_display);
				}

				// 方法2: 使用角点信息推断距离
				double width = distanceBetweenCorners(det, 0, 1);
				double height = distanceBetweenCorners(det, 1, 2);
				double avg_size = (width + height) / 2.0;
				
				std::cout << "相对距离(标签像素尺寸): " << avg_size << " 像素" << std::endl;

				if(avg_size > config.relative_distance){
					std::string message = std::to_string(config.node_id) + ":" + id_to_display;
        
					zmq::message_t zmq_msg(message.size());
					memcpy(zmq_msg.data(), message.c_str(), message.size());
					
					try {
						socket.send(zmq_msg, zmq::send_flags::none);
						std::cout << "节点 " << config.node_id << " 发送: " << message << std::endl;
					} catch (const zmq::error_t& e) {
						std::cerr << "发送失败: " << e.what() << std::endl;
					}						
				}
		
			}

			if(id_display_timer.elapsed_ms() > 3000){
				// display id 3 second

				id_display_timer.stop();
				if(config.open_preview){
					displayCenteredText("X");
				}
			}

			// 释放检测结果
			apriltag_detections_destroy(detections);


			// 创建可调整大小的窗口
			if(config.open_preview){
				cv::namedWindow("JPEG Viewer", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
			}

			// // 显示图像
			// cv::imshow("JPEG Viewer", frame);
			// cv::imshow("JPEG Viewer", result_image);

			// // 打印图像信息
			// std::cout << "成功显示图像: "
			//           << result_image.cols << "x" << result_image.rows 
			//           << " 通道数: " << result_image.channels() << std::endl;

			timer.stop();
			std::cout << "detection is " << 1000 / timer.elapsed_ms() << " fps\n";

			// 等待按键
			auto key = cv::waitKey(1);
			if(key == 27){
				//escpae key 退出
				EXIT = 0;
				// 销毁窗口
				cv::destroyAllWindows();
			}

			// if(pFrame->index >= 1000)
			// EXIT = 0;

			// fprintf(stderr,"pFrame->index:%02d PixFormat.u_Fps.:%d\r\n",pFrame->index,pFrame->PixFormat.u_Fps);
			TST_USBCam_SAVE_FRAME_RES(pUSBCam,pFrame);
		}
	}

	// 清理资源
	apriltag_detector_destroy(td);
	tag36h11_destroy(tf);
	cv::destroyAllWindows();

	return 0;

}

