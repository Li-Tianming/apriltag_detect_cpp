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

using namespace cv;

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
    Cfg.u_Width = 1280;
    Cfg.u_Height = 800;
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

int main(int argc, char *argv[])
{

    signal(SIGINT,SIG_QUIT);
    signal(SIGQUIT,SIG_QUIT);

    int32_t fd;
    
    /************************************************
    video stream control
    ************************************************/

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


    void* pUSBCam =
    TST_USBCam_CREATE_DEVICE_POINT     (pdevinfo[0]);

    fd = open(pdevinfo[0].Device_Path, O_RDWR | O_NONBLOCK, 0);
    
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

    while(EXIT)
    {
        Frame_Buffer_Data*pFrame = TST_USBCam_GET_FRAME_BUFF(pUSBCam,0);

        if(pFrame != NULL)
        {
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
            }

            // 释放检测结果
            apriltag_detections_destroy(detections);


            // 创建可调整大小的窗口
            cv::namedWindow("JPEG Viewer", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
            
            // // 显示图像
            cv::imshow("JPEG Viewer", frame);
            // cv::imshow("JPEG Viewer", result_image);
            
            // // 打印图像信息
            // std::cout << "成功显示图像: "
            //           << result_image.cols << "x" << result_image.rows 
            //           << " 通道数: " << result_image.channels() << std::endl;
            
            // 等待按键
            // std::cout << "按任意键退出..." << std::endl;
            auto key = cv::waitKey(1);
            if(key == 27){
                //escpae key
                EXIT = 0;
                // 销毁窗口
                cv::destroyAllWindows();
            }

            // if(pFrame->index >= 1000)
            // EXIT = 0;

            fprintf(stderr,"pFrame->index:%02d PixFormat.u_Fps.:%d\r\n",pFrame->index,pFrame->PixFormat.u_Fps);
            TST_USBCam_SAVE_FRAME_RES(pUSBCam,pFrame);
        }
    }

    // 清理资源
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
    cv::destroyAllWindows();

    return 0;

}

