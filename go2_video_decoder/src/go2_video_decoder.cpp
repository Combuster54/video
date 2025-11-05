#include "go2_video_decoder/go2_video_decoder.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <cstring>
#include <iomanip>

namespace go2_video_decoder {

Go2VideoDecoder::Go2VideoDecoder() 
  : pipeline_(nullptr)
  , appsrc_(nullptr)
  , appsink_(nullptr)
  , bus_(nullptr)
  , is_active_(false)
  , is_initialized_(false)
  , is_synced_(false)
  , sps_seen_(false)
  , pps_seen_(false)
  , sps_pps_sent_(false)
  , width_(0)
  , height_(0)
  , fps_(30.0)
{
  // Inicializar GStreamer
  gst_init(nullptr, nullptr);
}

Go2VideoDecoder::~Go2VideoDecoder() {
  stop();
  cleanup();
}

bool Go2VideoDecoder::initialize(int width, int height) {
  if (is_initialized_.load()) {
    cleanup();
  }

  width_ = width;
  height_ = height;
  
  if (!buildPipeline()) {
    std::cerr << "Error: No se pudo construir el pipeline de GStreamer" << std::endl;
    return false;
  }

  is_initialized_.store(true);
  std::cout << "Decodificador H.264 inicializado correctamente (appsrc)" << std::endl;
  return true;
}

bool Go2VideoDecoder::buildPipeline() {
  // Pipeline GStreamer para decodificar H.264 usando appsrc
  // appsrc -> h264parse -> avdec_h264 -> videoconvert -> appsink

    std::string pipeline_str =
    "appsrc name=source is-live=true format=time ! "
    "h264parse config-interval=1 disable-passthrough=true ! "
    "queue ! avdec_h264 ! queue ! videoconvert ! "
    "video/x-raw,format=BGR ! "
    "appsink name=sink emit-signals=true max-buffers=1 drop=false sync=false";

  std::cout << "\n=== CONSTRUYENDO PIPELINE GSTREAMER ===" << std::endl;
  std::cout << "Pipeline: " << pipeline_str << std::endl;
  std::cout << "========================================\n" << std::endl;
  
  pipeline_ = gst_parse_launch(pipeline_str.c_str(), nullptr);
  if (!pipeline_) {
    std::cerr << "Error: No se pudo crear el pipeline de GStreamer" << std::endl;
    return false;
  }

  // Obtener appsrc
  appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "source");
  if (!appsrc_) {
    std::cerr << "Error: No se pudo obtener appsrc" << std::endl;
    return false;
  }

  // Configurar appsrc con mejores opciones
  g_object_set(G_OBJECT(appsrc_),
    "format", GST_FORMAT_TIME,
    "is-live", TRUE,
    "do-timestamp", TRUE,
    "block", FALSE,
    "max-bytes", 0,
    "min-percent", 0,
    nullptr);

  // Configurar caps para appsrc
  GstCaps* caps = gst_caps_new_simple("video/x-h264",
    "stream-format", G_TYPE_STRING, "byte-stream",
    "alignment", G_TYPE_STRING, "nal",
    nullptr);
  gst_app_src_set_caps(GST_APP_SRC(appsrc_), caps);
  gst_caps_unref(caps);

  // Obtener appsink
  appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
  if (!appsink_) {
    std::cerr << "Error: No se pudo obtener appsink" << std::endl;
    return false;
  }

  // Configurar callback para nuevos frames
  GstAppSinkCallbacks callbacks;
  callbacks.eos = nullptr;
  callbacks.new_preroll = nullptr;
  callbacks.new_sample = onNewFrame;
  gst_app_sink_set_callbacks(GST_APP_SINK(appsink_), &callbacks, this, nullptr);

  // Configurar bus para mensajes
  bus_ = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));
  gst_bus_add_watch(bus_, busCall, this);

  return true;
}

bool Go2VideoDecoder::pushH264Data(const uint8_t* data, size_t size) {
  if (!is_initialized_.load() || !appsrc_ || size == 0) {
    static int error_count = 0;
    if (++error_count % 100 == 0) {
      std::cerr << "[pushH264Data] Error: decodificador no inicializado o datos vacíos (count: " 
                << error_count << ")" << std::endl;
    }
    return false;
  }

  std::lock_guard<std::mutex> lock(push_mutex_);
  
  // Log detallado del primer byte para debugging (solo los primeros 100 para no saturar)
  static int push_count = 0;
  static bool first_log = true;
  if (first_log || push_count < 100) {
    if (push_count == 0) {
      std::cout << "\n=== PRIMEROS DATOS H.264 RECIBIDOS ===" << std::endl;
      std::cout << "Tamaño: " << size << " bytes" << std::endl;
      if (size >= 4) {
        std::cout << "Primeros 4 bytes (hex): ";
        for (size_t i = 0; i < std::min(size, size_t(4)); i++) {
          std::cout << "0x" << std::hex << std::setfill('0') << std::setw(2) 
                    << static_cast<int>(data[i]) << " ";
        }
        std::cout << std::dec << std::endl;
      }
      std::cout << "=====================================\n" << std::endl;
      first_log = false;
    }
    push_count++;
  }

  // Establecer flags y timestamps
  // Detectar tipo de NAL unit para marcar correctamente los frames
  // Los datos H.264 pueden venir con start codes (0x00 0x00 0x00 0x01 o 0x00 0x00 0x01)
  bool is_keyframe = false;
  size_t offset = 0;
  
  // Buscar start code (si existe)
  if (size >= 4 && data[0] == 0x00 && data[1] == 0x00) {
    if (data[2] == 0x00 && data[3] == 0x01) {
      offset = 4;  // Start code de 4 bytes
    } else if (data[2] == 0x01) {
      offset = 3;  // Start code de 3 bytes
    }
  }
  
  // Escaneo completo para detectar NALs presentes en el buffer (maneja múltiples NALs por mensaje)
  bool contains_sps = false, contains_pps = false, contains_idr = false;
  size_t i_scan = 0;
  while (i_scan + 3 < size) {
    size_t sc_len = 0;
    if (i_scan + 4 <= size && data[i_scan] == 0x00 && data[i_scan+1] == 0x00 && data[i_scan+2] == 0x00 && data[i_scan+3] == 0x01) {
      sc_len = 4;
    } else if (i_scan + 3 <= size && data[i_scan] == 0x00 && data[i_scan+1] == 0x00 && data[i_scan+2] == 0x01) {
      sc_len = 3;
    }
    if (sc_len == 0) { i_scan++; continue; }
    size_t nal_hdr = i_scan + sc_len;
    if (nal_hdr >= size) break;
    uint8_t nal_type_scan = data[nal_hdr] & 0x1F;
    if (nal_type_scan == 7) contains_sps = true;
    if (nal_type_scan == 8) contains_pps = true;
    if (nal_type_scan == 5) contains_idr = true;
    // Extraer y cachear SPS/PPS (incluyendo start code) hasta el siguiente start code o fin
    if (nal_type_scan == 7 || nal_type_scan == 8) {
      size_t next = nal_hdr + 1;
      // Buscar próximo start code
      while (next + 3 < size) {
        if ((next + 4 <= size && data[next] == 0x00 && data[next+1] == 0x00 && data[next+2] == 0x00 && data[next+3] == 0x01) ||
            (next + 3 <= size && data[next] == 0x00 && data[next+1] == 0x00 && data[next+2] == 0x01)) {
          break;
        }
        next++;
      }
      const uint8_t* start_ptr = &data[i_scan];
      size_t nal_len = (next > i_scan) ? (next - i_scan) : (size - i_scan);
      if (nal_type_scan == 7) {
        cached_sps_.assign(start_ptr, start_ptr + nal_len);
      } else {
        cached_pps_.assign(start_ptr, start_ptr + nal_len);
      }
    }
    i_scan = nal_hdr + 1;
  }

  if (contains_sps) sps_seen_.store(true);
  if (contains_pps) pps_seen_.store(true);
  if (contains_idr) { is_keyframe = true; is_synced_.store(true); }

  // Log básico del primer NAL detectado en el paquete
  if (size > offset) {
    uint8_t nal_type = (data[offset] & 0x1F);
    static int nal_log_count = 0;
    if (nal_log_count < 20) {
      const char* nal_names[] = {
        "Unspecified", "Non-IDR", "Data A", "Data B", "Data C",
        "IDR", "SEI", "SPS", "PPS", "AUD", "End of sequence", "End of stream"
      };
      const char* nal_name = (nal_type < 12) ? nal_names[nal_type] : "Unknown";
      std::cout << "[NAL] Tipo: " << static_cast<int>(nal_type)
                << " (" << nal_name << ")"
                << ", Tamaño: " << size << " bytes" << std::endl;
      nal_log_count++;
    }
  }

  // Gateo: hasta que llegue el primer IDR, solo permitimos buffers con SPS/PPS/IDR
  if (!is_synced_.load()) {
    static int unsynced_count = 0;
    if (contains_sps && !sps_seen_.load()) {
      std::cout << "[SYNC] SPS detectado por primera vez" << std::endl;
    }
    if (contains_pps && !pps_seen_.load()) {
      std::cout << "[SYNC] PPS detectado por primera vez" << std::endl;
    }
    if (contains_idr) {
      std::cout << "[SYNC] IDR detectado. Sincronización habilitada." << std::endl;
    }
    if (!(contains_sps || contains_pps || contains_idr)) {
      unsynced_count++;
      if (unsynced_count <= 20 || unsynced_count % 200 == 0) {
        std::cout << "[SYNC] Descartando buffer sin SPS/PPS/IDR (size=" << size 
                  << ") mientras esperamos IDR... (count=" << unsynced_count << ")" << std::endl;
      }
      return true; // descartar silenciosamente antes de alocar buffer
    }
  }

  // Si llegó un IDR y aún no enviamos SPS/PPS, prepéndelos explícitamente antes del IDR
  if (contains_idr && !sps_pps_sent_.load()) {
    auto push_vec = [&](const std::vector<uint8_t>& vec, const char* tag) {
      if (vec.empty()) return;
      GstBuffer* b = gst_buffer_new_allocate(nullptr, vec.size(), nullptr);
      if (!b) return;
      GstMapInfo m;
      if (gst_buffer_map(b, &m, GST_MAP_WRITE)) {
        memcpy(m.data, vec.data(), vec.size());
        gst_buffer_unmap(b, &m);
      }
      GST_BUFFER_PTS(b) = GST_CLOCK_TIME_NONE;
      GST_BUFFER_DTS(b) = GST_CLOCK_TIME_NONE;
      GST_BUFFER_DURATION(b) = GST_CLOCK_TIME_NONE;
      gst_app_src_push_buffer(GST_APP_SRC(appsrc_), b);
      std::cout << "[SYNC] " << tag << " enviado antes de IDR (" << vec.size() << " bytes)" << std::endl;
    };
    if (!cached_sps_.empty() || !cached_pps_.empty()) {
      push_vec(cached_sps_, "SPS");
      push_vec(cached_pps_, "PPS");
      sps_pps_sent_.store(true);
    }
  }

  // Crear buffer de GStreamer (tras validar gateo)
  GstBuffer* buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
  if (!buffer) {
    std::cerr << "Error: No se pudo crear buffer de GStreamer" << std::endl;
    return false;
  }

  // Copiar datos al buffer
  GstMapInfo map;
  if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
    memcpy(map.data, data, size);
    gst_buffer_unmap(buffer, &map);
  } else {
    gst_buffer_unref(buffer);
    return false;
  }
  
  // Marcar flags según el tipo de frame
  if (!is_keyframe) {
    GST_BUFFER_FLAG_SET(buffer, GST_BUFFER_FLAG_DELTA_UNIT);
  } else {
    GST_BUFFER_FLAG_UNSET(buffer, GST_BUFFER_FLAG_DELTA_UNIT);
  }

  // Usar timestamp automático (do-timestamp está activado)
  GST_BUFFER_PTS(buffer) = GST_CLOCK_TIME_NONE;
  GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
  GST_BUFFER_DURATION(buffer) = GST_CLOCK_TIME_NONE;

  // Enviar buffer al pipeline
  GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
  
  // Log del resultado (solo errores y algunos éxitos)
  static int success_count = 0;
  static int error_count = 0;
  
  if (ret != GST_FLOW_OK && ret != GST_FLOW_FLUSHING) {
    error_count++;
    const char* ret_names[] = {
      "FLOW_OK", "FLOW_NOT_LINKED", "FLOW_FLUSHING", "FLOW_EOS", 
      "FLOW_NOT_NEGOTIATED", "FLOW_ERROR", "FLOW_NOT_SUPPORTED", "FLOW_CUSTOM_ERROR"
    };
    const char* ret_name = (ret >= GST_FLOW_OK && ret <= GST_FLOW_CUSTOM_ERROR) 
                            ? ret_names[ret] : "UNKNOWN";
    std::cerr << "[ERROR pushH264Data] GstFlowReturn: " << ret 
              << " (" << ret_name << "), Buffers fallidos: " << error_count << std::endl;
    if (ret == GST_FLOW_NOT_LINKED) {
      std::cerr << "[HINT] NOT_LINKED: suele indicar falta de SPS/PPS/IDR o negociación pendiente."
                << std::endl;
      std::cerr << "       Estado: is_synced=" << (is_synced_.load() ? 1 : 0)
                << ", sps_seen=" << (sps_seen_.load() ? 1 : 0)
                << ", pps_seen=" << (pps_seen_.load() ? 1 : 0) << std::endl;
    }
    return false;
  }
  
  success_count++;
  if (success_count <= 10 || success_count % 1000 == 0) {
    std::cout << "[OK pushH264Data] Buffer enviado correctamente. Total: " 
              << success_count << ", Tamaño: " << size << " bytes" << std::endl;
  }

  return true;
}

GstFlowReturn Go2VideoDecoder::onNewFrame(GstAppSink* appsink, gpointer user_data) {
  Go2VideoDecoder* decoder = static_cast<Go2VideoDecoder*>(user_data);
  
  GstSample* sample = gst_app_sink_pull_sample(appsink);
  if (!sample) {
    static int error_count = 0;
    if (++error_count % 100 == 0) {
      std::cerr << "[onNewFrame] Error: No se pudo obtener sample (count: " 
                << error_count << ")" << std::endl;
    }
    return GST_FLOW_ERROR;
  }

  GstBuffer* buffer = gst_sample_get_buffer(sample);
  GstCaps* caps = gst_sample_get_caps(sample);
  GstStructure* structure = gst_caps_get_structure(caps, 0);
  
  gint width, height;
  gst_structure_get_int(structure, "width", &width);
  gst_structure_get_int(structure, "height", &height);
  
  decoder->width_ = width;
  decoder->height_ = height;

  // Log de frames decodificados
  static int frame_count = 0;
  static bool first_frame = true;
  if (first_frame || frame_count < 10 || frame_count % 100 == 0) {
    std::cout << "[onNewFrame] Frame decodificado #" << frame_count 
              << ": " << width << "x" << height << " píxeles" << std::endl;
    if (first_frame) {
      std::cout << "=== PRIMER FRAME DECODIFICADO EXITOSAMENTE ===" << std::endl;
      first_frame = false;
    }
  }
  frame_count++;

  GstMapInfo map;
  if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
    cv::Mat frame(height, width, CV_8UC3, (char*)map.data, cv::Mat::AUTO_STEP);
    cv::Mat frame_copy = frame.clone();
    
    {
      std::lock_guard<std::mutex> lock(decoder->frame_mutex_);
      decoder->latest_frame_ = frame_copy;
    }
    
    // Llamar callback si está configurado
    if (decoder->frame_callback_) {
      decoder->frame_callback_(frame_copy);
    }
    
    gst_buffer_unmap(buffer, &map);
  }

  gst_sample_unref(sample);
  return GST_FLOW_OK;
}

gboolean Go2VideoDecoder::busCall(GstBus* bus, GstMessage* msg, gpointer data) {
  Go2VideoDecoder* decoder = static_cast<Go2VideoDecoder*>(data);
  
  switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_EOS:
      std::cout << "[GStreamer] Mensaje: Fin del stream (EOS)" << std::endl;
      decoder->is_active_.store(false);
      break;
      
    case GST_MESSAGE_ERROR: {
      gchar* debug;
      GError* error;
      gst_message_parse_error(msg, &error, &debug);
      std::cerr << "\n[GStreamer ERROR] " << error->message << std::endl;
      if (debug) {
        std::cerr << "[GStreamer DEBUG] " << debug << std::endl;
      }
      g_free(debug);
      g_error_free(error);
      decoder->is_active_.store(false);
      std::cerr << "[CTX] Flags: is_synced=" << (decoder->is_synced_.load()?1:0)
                << ", sps_seen=" << (decoder->sps_seen_.load()?1:0)
                << ", pps_seen=" << (decoder->pps_seen_.load()?1:0) << std::endl;
      if (decoder->pipeline_) {
        GstState s = GST_STATE_NULL, p = GST_STATE_NULL;
        gst_element_get_state(decoder->pipeline_, &s, &p, 0);
        const char* state_names[] = {"VOID_PENDING", "NULL", "READY", "PAUSED", "PLAYING"};
        std::cerr << "[CTX] Pipeline state: " << state_names[s] << ", pending: " << state_names[p] << std::endl;
      }
      if (decoder->pipeline_) {
        gst_element_set_state(decoder->pipeline_, GST_STATE_NULL);
      }
      break;
    }
    
    case GST_MESSAGE_WARNING: {
      gchar* debug;
      GError* error;
      gst_message_parse_warning(msg, &error, &debug);
      static int warning_count = 0;
      if (++warning_count <= 10 || warning_count % 100 == 0) {
        std::cerr << "[GStreamer WARNING] " << error->message << std::endl;
        if (debug) {
          std::cerr << "[GStreamer DEBUG] " << debug << std::endl;
        }
      }
      g_free(debug);
      g_error_free(error);
      break;
    }
    
    case GST_MESSAGE_STATE_CHANGED: {
      GstState old_state, new_state, pending_state;
      gst_message_parse_state_changed(msg, &old_state, &new_state, &pending_state);
      const gchar* src_name = GST_OBJECT_NAME(msg->src);
      if (g_str_has_prefix(src_name, "pipeline") || g_str_has_prefix(src_name, "source")) {
        const char* state_names[] = {"VOID_PENDING", "NULL", "READY", "PAUSED", "PLAYING"};
        std::cout << "[GStreamer] Estado cambiado (" << src_name << "): " 
                  << state_names[old_state] << " -> " << state_names[new_state] << std::endl;
      }
      break;
    }
    
    default:
      break;
  }
  
  return TRUE;
}

bool Go2VideoDecoder::getNextFrame(cv::Mat& frame) {
  if (!is_initialized_.load()) {
    return false;
  }

  std::lock_guard<std::mutex> lock(frame_mutex_);
  if (latest_frame_.empty()) {
    return false;
  }

  frame = latest_frame_.clone();
  return true;
}

void Go2VideoDecoder::setFrameCallback(FrameCallback callback) {
  frame_callback_ = callback;
}

void Go2VideoDecoder::start() {
  if (is_active_.load()) {
    std::cout << "El decodificador ya está activo" << std::endl;
    return;
  }

  if (!is_initialized_.load()) {
    std::cerr << "Error: El decodificador no está inicializado" << std::endl;
    return;
  }

  is_active_.store(true);
  
  // Iniciar pipeline
  GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    std::cerr << "Error: No se pudo iniciar el pipeline" << std::endl;
    is_active_.store(false);
    return;
  }

  processing_thread_ = std::make_unique<std::thread>(&Go2VideoDecoder::processVideo, this);
  std::cout << "Procesamiento de video H.264 iniciado" << std::endl;
}

void Go2VideoDecoder::stop() {
  // Forzar estado NULL siempre para evitar CRITICAL al liberar
  is_active_.store(false);
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
  }
  if (processing_thread_ && processing_thread_->joinable()) {
    processing_thread_->join();
    processing_thread_.reset();
  }
  std::cout << "Procesamiento de video detenido" << std::endl;
}

bool Go2VideoDecoder::isActive() const {
  return is_active_.load();
}

bool Go2VideoDecoder::getVideoInfo(int& width, int& height, double& fps) const {
  if (!is_initialized_.load()) {
    return false;
  }

  width = width_;
  height = height_;
  fps = fps_;
  
  return (width > 0 && height > 0);
}

void Go2VideoDecoder::processVideo() {
  // Ejecutar iteraciones del contexto de GStreamer mientras esté activo
  while (is_active_.load()) {
    // Procesar mensajes del bus
    if (bus_) {
      GstMessage* msg = gst_bus_pop(bus_);
      if (msg) {
        busCall(bus_, msg, this);
        gst_message_unref(msg);
      }
    }
    
    // Iterar el contexto principal
    GMainContext* context = g_main_context_default();
    while (g_main_context_pending(context)) {
      g_main_context_iteration(context, FALSE);
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void Go2VideoDecoder::cleanup() {
  if (appsrc_) {
    gst_object_unref(appsrc_);
    appsrc_ = nullptr;
  }

  if (appsink_) {
    gst_object_unref(appsink_);
    appsink_ = nullptr;
  }
  
  if (bus_) {
    gst_object_unref(bus_);
    bus_ = nullptr;
  }
  
  if (pipeline_) {
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }
  
  is_initialized_.store(false);
}

} // namespace go2_video_decoder

