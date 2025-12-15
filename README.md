# Pozisyon-Korumali-Drone



Bu proje, ESP32 kullanılarak geliştirilmiş, MPU9250, PMW3901 ve TOF (VL53L1X / TOF400) sensörleri ile çalışan, pozisyon koruma odaklı bir drone sistemidir.



Projede herhangi bir fiziksel kumanda bulunmamaktadır. Drone, ESP32’nin kendi oluşturduğu Wi-Fi ağı üzerinden yayınlanan bir web arayüzü aracılığıyla kontrol edilmektedir. Kullanıcı, bu ağa bağlanarak web arayüzü üzerinden HTTP istekleri ile drone’a komut göndermektedir.



--------------------------------------------------



Genel Çalışma Mantığı



Drone, web arayüzünde bulunan kontrolcü sayesinde normal bir ticari drone gibi yukarı, aşağı, sağa ve sola hareket edebilmektedir. Kullanıcıdan gelen bu komutlar doğrudan uçuş kontrol sistemine iletilir ve motorlar buna göre kontrol edilir.



Bu projenin temel amacı pozisyon koruma özelliğini sağlamaktır. Drone üzerinde bulunan sensörler sayesinde konum ve yükseklik bilgileri sürekli olarak ölçülmektedir. Web arayüzünden herhangi bir hareket komutu gelmediği durumda, drone’un kendi kendine yaptığı hareketler sürüklenme olarak algılanır.



Sürüklenme algılandığında sistem, en son kaydedilen konumu referans alır ve drone’un bu konuma geri dönmesi için motorları ve uçuş dengesini otomatik olarak ayarlar. Bu sayede drone, bulunduğu konumu mümkün olduğunca sabit tutmaya çalışır.



--------------------------------------------------



Kullanılan Sensörler ve Bileşenler



\- ESP32

\- MPU9250 (IMU)

\- PMW3901 (Optical Flow)

\- TOF sensörü (VL53L1X / TOF400)

\- Web tabanlı kontrol arayüzü

\- HTTP tabanlı kontrol mekanizması



--------------------------------------------------



Notlar



Bu proje, eğitim ve yazılım geliştirme amacıyla hazırlanmıştır.  

Pozisyon koruma, sensör verilerinin birlikte değerlendirilmesiyle yazılımsal olarak sağlanmaktadır.  

Projede fiziksel kumanda veya harici kontrol donanımı bulunmamaktadır.



