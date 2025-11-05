#ifndef GO2_VIDEO_DECODER_HPP
#define GO2_VIDEO_DECODER_HPP

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>
#include <vector>

namespace go2_video_decoder {

class Go2VideoDecoder {
public:
  using FrameCallback = std::function<void(const cv::Mat&)>;

  Go2VideoDecoder();
  ~Go2VideoDecoder();

  /**
   * @brief Inicializa el decodificador de video H.264 con GStreamer usando appsrc
   * @param width Ancho esperado del video (opcional, se detecta automáticamente)
   * @param height Alto esperado del video (opcional, se detecta automáticamente)
   * @return true si la inicialización fue exitosa
   */
  bool initialize(int width = 1920, int height = 1080);

  /**
   * @brief Envía datos H.264 al pipeline para decodificar
   * @param data Puntero a los datos H.264
   * @param size Tamaño de los datos en bytes
   * @return true si se enviaron correctamente
   */
  bool pushH264Data(const uint8_t* data, size_t size);

  /**
   * @brief Obtiene el siguiente frame del video decodificado
   * @param frame Frame de salida en formato BGR
   * @return true si se obtuvo un frame válido
   */
  bool getNextFrame(cv::Mat& frame);

  /**
   * @brief Establece un callback para recibir frames cuando están disponibles
   * @param callback Función que se llamará con cada frame
   */
  void setFrameCallback(FrameCallback callback);

  /**
   * @brief Inicia el procesamiento del video en un hilo separado
   */
  void start();

  /**
   * @brief Detiene el procesamiento del video
   */
  void stop();

  /**
   * @brief Verifica si el decodificador está activo
   * @return true si está activo
   */
  bool isActive() const;

  /**
   * @brief Obtiene información del video
   * @param width Ancho del video
   * @param height Alto del video
   * @param fps Frames por segundo
   * @return true si se obtuvieron los datos correctamente
   */
  bool getVideoInfo(int& width, int& height, double& fps) const;

private:
  GstElement* pipeline_;
  GstElement* appsrc_;
  GstElement* appsink_;
  GstBus* bus_;
  
  std::atomic<bool> is_active_;
  std::atomic<bool> is_initialized_;
  std::atomic<bool> is_synced_;
  std::atomic<bool> sps_seen_;
  std::atomic<bool> pps_seen_;
  std::atomic<bool> sps_pps_sent_;
  std::unique_ptr<std::thread> processing_thread_;
  std::mutex frame_mutex_;
  std::mutex push_mutex_;
  std::vector<uint8_t> cached_sps_;
  std::vector<uint8_t> cached_pps_;
  
  cv::Mat latest_frame_;
  FrameCallback frame_callback_;
  
  int width_;
  int height_;
  double fps_;
  
  void processVideo();
  static GstFlowReturn onNewFrame(GstAppSink* appsink, gpointer user_data);
  static gboolean busCall(GstBus* bus, GstMessage* msg, gpointer data);
  bool buildPipeline();
  void cleanup();
};

} // namespace go2_video_decoder

#endif // GO2_VIDEO_DECODER_HPP

