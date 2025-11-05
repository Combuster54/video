#include "go2_video_decoder/go2_video_decoder.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <unitree_go/msg/go2_front_video_data.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <memory>
#include <mutex>
#include <sstream>

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
  // Analiza un buffer H.264 y genera un resumen de NALs encontrados
  static std::string summarizeH264NALs(const uint8_t* data, size_t size) {
    if (!data || size < 4) return "(vacío)";
    size_t i = 0;
    int total_nals = 0;
    int cnt_nonidr = 0, cnt_idr = 0, cnt_sps = 0, cnt_pps = 0, cnt_sei = 0, cnt_aud = 0;
    int first_nal = -1;
    std::ostringstream seq;
    int seq_printed = 0;
    auto nalName = [](int t) {
      switch (t) {
        case 1: return "Non-IDR"; case 5: return "IDR"; case 6: return "SEI";
        case 7: return "SPS"; case 8: return "PPS"; case 9: return "AUD";
        default: return "Other";
      }
    };
    while (i + 3 < size) {
      size_t sc_len = 0;
      if (i + 4 <= size && data[i]==0x00 && data[i+1]==0x00 && data[i+2]==0x00 && data[i+3]==0x01) sc_len = 4;
      else if (i + 3 <= size && data[i]==0x00 && data[i+1]==0x00 && data[i+2]==0x01) sc_len = 3;
      if (sc_len == 0) { i++; continue; }
      size_t nal_hdr = i + sc_len;
      if (nal_hdr >= size) break;
      int nal_type = data[nal_hdr] & 0x1F;
      if (first_nal < 0) first_nal = nal_type;
      total_nals++;
      if (nal_type == 1) cnt_nonidr++;
      else if (nal_type == 5) cnt_idr++;
      else if (nal_type == 7) cnt_sps++;
      else if (nal_type == 8) cnt_pps++;
      else if (nal_type == 6) cnt_sei++;
      else if (nal_type == 9) cnt_aud++;
      if (seq_printed < 10) {
        if (seq_printed > 0) seq << ",";
        seq << nal_type;
        seq_printed++;
      }
      i = nal_hdr + 1;
    }
    std::ostringstream out;
    out << "NALs total=" << total_nals
        << " | primeros=[" << seq.str() << "]"
        << " | first=" << (first_nal >= 0 ? std::to_string(first_nal) : std::string("-"))
        << " | SPS=" << cnt_sps << ", PPS=" << cnt_pps << ", IDR=" << cnt_idr
        << ", NonIDR=" << cnt_nonidr << ", SEI=" << cnt_sei << ", AUD=" << cnt_aud;
    return out.str();
  }

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

    // Log detallado del flujo H.264 en cada mensaje
    static int msg_count = 0;
    static int log_count = 0;
    msg_count++;
    
    // Siempre imprimimos un resumen compacto por mensaje para diagnóstico
    std::string nal_summary = summarizeH264NALs(video_data, video_size);
    RCLCPP_INFO(this->get_logger(),
      "[ROS2] Msg #%d: %s, %zu bytes, time_frame=%lu | %s",
      msg_count, video_type.c_str(), video_size, msg->time_frame, nal_summary.c_str());

    // Enviar datos H.264 al pipeline de GStreamer
    if (!decoder_.pushH264Data(video_data, video_size)) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Error al enviar datos H.264 (%s) al decodificador", video_type.c_str());
    } else {
      RCLCPP_DEBUG(this->get_logger(),
        "Datos H.264 (%s) enviados correctamente al pipeline", video_type.c_str());
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

