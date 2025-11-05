#include "go2_video_decoder/go2_video_decoder.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <unitree_go/msg/go2_front_video_data.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <memory>
#include <mutex>

using std::placeholders::_1;
using unitree_go::msg::Go2FrontVideoData;

class Go2VideoDecoderNode : public rclcpp::Node {
public:
  Go2VideoDecoderNode() : Node("go2_video_decoder_node") {
    // Declarar parámetros
    this->declare_parameter<std::string>("video_sub_topic", "/frontvideostream");
    this->declare_parameter<std::string>("image_pub_topic", "camera");
    this->declare_parameter<std::string>("frame_id", "camera_frame");
    this->declare_parameter<int>("width", 1920);
    this->declare_parameter<int>("height", 1080);
    
    // Obtener parámetros
    std::string video_sub_topic = this->get_parameter("video_sub_topic").as_string();
    std::string image_pub_topic = this->get_parameter("image_pub_topic").as_string();
    std::string frame_id = this->get_parameter("frame_id").as_string();
    int width = this->get_parameter("width").as_int();
    int height = this->get_parameter("height").as_int();
    
    frame_id_ = frame_id;
    
    // Crear subscriber para Go2FrontVideoData
    video_subscriber_ = this->create_subscription<Go2FrontVideoData>(
      video_sub_topic, 10,
      std::bind(&Go2VideoDecoderNode::videoDataCallback, this, _1));
    
    // Crear publisher para sensor_msgs/Image
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      image_pub_topic, 10);
    
    RCLCPP_INFO(this->get_logger(), "Inicializando decodificador H.264 con GStreamer...");
    RCLCPP_INFO(this->get_logger(), "Suscrito a: %s", video_sub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publicando en: %s", image_pub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Resolución esperada: %dx%d", width, height);
    
    // Inicializar decodificador con appsrc
    if (!decoder_.initialize(width, height)) {
      RCLCPP_ERROR(this->get_logger(), "Error al inicializar el decodificador");
      return;
    }
    
    // Configurar callback para frames decodificados
    decoder_.setFrameCallback(std::bind(
      &Go2VideoDecoderNode::onFrameReceived, this, std::placeholders::_1));
    
    // Iniciar decodificador
    decoder_.start();
    
    if (!decoder_.isActive()) {
      RCLCPP_ERROR(this->get_logger(), "Error al iniciar el decodificador");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Decodificador iniciado correctamente");
  }
  
  ~Go2VideoDecoderNode() {
    decoder_.stop();
  }

private:
  void videoDataCallback(const Go2FrontVideoData::SharedPtr msg) {
    // Extraer datos H.264 del mensaje
    // Verificar si alguno de los streams de video tiene datos
    bool has_data = false;
    const uint8_t* video_data = nullptr;
    size_t video_size = 0;
    std::string video_type = "";

    // Prioridad: 720p -> 360p -> 180p
    if (!msg->data.empty()) {
      video_data = msg->data.data();
      video_size = msg->data.size();
      video_type = "720p";
      has_data = true;
    }

    if (!has_data) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Todos los streams de video están vacíos (720p, 360p, 180p)");
      return;
    }

    // Log detallado (solo algunos mensajes)
    static int msg_count = 0;
    static int log_count = 0;
    msg_count++;
    
    if (log_count < 10 || msg_count % 1000 == 0) {
      RCLCPP_INFO(this->get_logger(),
        "[ROS2] Mensaje #%d recibido: %s, %zu bytes, time_frame: %lu",
        msg_count, video_type.c_str(), video_size, msg->time_frame);
      log_count++;
    }

    // Enviar datos H.264 al pipeline de GStreamer
    if (!decoder_.pushH264Data(video_data, video_size)) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Error al enviar datos H.264 (%s) al decodificador", video_type.c_str());
    } else {
      // Log solo ocasional para no saturar
      if (log_count < 10 || msg_count % 1000 == 0) {
        RCLCPP_DEBUG(this->get_logger(),
          "Datos H.264 (%s) enviados correctamente al pipeline", video_type.c_str());
      }
    }
  }

  void onFrameReceived(const cv::Mat& frame) {
    if (frame.empty()) {
      return;
    }

    try {
      // Convertir cv::Mat a sensor_msgs::Image y publicar
      std_msgs::msg::Header header;
      header.stamp = this->now();
      header.frame_id = frame_id_;
      
      sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(
        header, "bgr8", frame).toImageMsg();
      
      image_publisher_->publish(*img_msg);
      
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Frame publicado: %dx%d", frame.cols, frame.rows);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), 
        "Error cv_bridge al convertir imagen: %s", e.what());
    }
  }
  
  go2_video_decoder::Go2VideoDecoder decoder_;
  rclcpp::Subscription<Go2FrontVideoData>::SharedPtr video_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  
  std::string frame_id_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<Go2VideoDecoderNode>();
  
  RCLCPP_INFO(node->get_logger(), "Nodo go2_video_decoder iniciado");
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}

