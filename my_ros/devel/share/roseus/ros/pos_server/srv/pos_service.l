;; Auto-generated. Do not edit!


(when (boundp 'pos_server::pos_service)
  (if (not (find-package "POS_SERVER"))
    (make-package "POS_SERVER"))
  (shadow 'pos_service (find-package "POS_SERVER")))
(unless (find-package "POS_SERVER::POS_SERVICE")
  (make-package "POS_SERVER::POS_SERVICE"))
(unless (find-package "POS_SERVER::POS_SERVICEREQUEST")
  (make-package "POS_SERVER::POS_SERVICEREQUEST"))
(unless (find-package "POS_SERVER::POS_SERVICERESPONSE")
  (make-package "POS_SERVER::POS_SERVICERESPONSE"))

(in-package "ROS")





(defclass pos_server::pos_serviceRequest
  :super ros::object
  :slots ())

(defmethod pos_server::pos_serviceRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass pos_server::pos_serviceResponse
  :super ros::object
  :slots (_x _y ))

(defmethod pos_server::pos_serviceResponse
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass pos_server::pos_service
  :super ros::object
  :slots ())

(setf (get pos_server::pos_service :md5sum-) "ff8d7d66dd3e4b731ef14a45d38888b6")
(setf (get pos_server::pos_service :datatype-) "pos_server/pos_service")
(setf (get pos_server::pos_service :request) pos_server::pos_serviceRequest)
(setf (get pos_server::pos_service :response) pos_server::pos_serviceResponse)

(defmethod pos_server::pos_serviceRequest
  (:response () (instance pos_server::pos_serviceResponse :init)))

(setf (get pos_server::pos_serviceRequest :md5sum-) "ff8d7d66dd3e4b731ef14a45d38888b6")
(setf (get pos_server::pos_serviceRequest :datatype-) "pos_server/pos_serviceRequest")
(setf (get pos_server::pos_serviceRequest :definition-)
      "---
float32 x
float32 y

")

(setf (get pos_server::pos_serviceResponse :md5sum-) "ff8d7d66dd3e4b731ef14a45d38888b6")
(setf (get pos_server::pos_serviceResponse :datatype-) "pos_server/pos_serviceResponse")
(setf (get pos_server::pos_serviceResponse :definition-)
      "---
float32 x
float32 y

")



(provide :pos_server/pos_service "ff8d7d66dd3e4b731ef14a45d38888b6")


