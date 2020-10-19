;; Auto-generated. Do not edit!


(when (boundp 'custom_messages::Sum)
  (if (not (find-package "CUSTOM_MESSAGES"))
    (make-package "CUSTOM_MESSAGES"))
  (shadow 'Sum (find-package "CUSTOM_MESSAGES")))
(unless (find-package "CUSTOM_MESSAGES::SUM")
  (make-package "CUSTOM_MESSAGES::SUM"))
(unless (find-package "CUSTOM_MESSAGES::SUMREQUEST")
  (make-package "CUSTOM_MESSAGES::SUMREQUEST"))
(unless (find-package "CUSTOM_MESSAGES::SUMRESPONSE")
  (make-package "CUSTOM_MESSAGES::SUMRESPONSE"))

(in-package "ROS")





(defclass custom_messages::SumRequest
  :super ros::object
  :slots (_a _b ))

(defmethod custom_messages::SumRequest
  (:init
   (&key
    ((:a __a) 0.0)
    ((:b __b) 0.0)
    )
   (send-super :init)
   (setq _a (float __a))
   (setq _b (float __b))
   self)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:b
   (&optional __b)
   (if __b (setq _b __b)) _b)
  (:serialization-length
   ()
   (+
    ;; float32 _a
    4
    ;; float32 _b
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _a
       (sys::poke _a (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _b
       (sys::poke _b (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _a
     (setq _a (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _b
     (setq _b (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass custom_messages::SumResponse
  :super ros::object
  :slots (_sum ))

(defmethod custom_messages::SumResponse
  (:init
   (&key
    ((:sum __sum) 0.0)
    )
   (send-super :init)
   (setq _sum (float __sum))
   self)
  (:sum
   (&optional __sum)
   (if __sum (setq _sum __sum)) _sum)
  (:serialization-length
   ()
   (+
    ;; float32 _sum
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _sum
       (sys::poke _sum (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _sum
     (setq _sum (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass custom_messages::Sum
  :super ros::object
  :slots ())

(setf (get custom_messages::Sum :md5sum-) "210a18f816d5b88becb133b82fae0c4c")
(setf (get custom_messages::Sum :datatype-) "custom_messages/Sum")
(setf (get custom_messages::Sum :request) custom_messages::SumRequest)
(setf (get custom_messages::Sum :response) custom_messages::SumResponse)

(defmethod custom_messages::SumRequest
  (:response () (instance custom_messages::SumResponse :init)))

(setf (get custom_messages::SumRequest :md5sum-) "210a18f816d5b88becb133b82fae0c4c")
(setf (get custom_messages::SumRequest :datatype-) "custom_messages/SumRequest")
(setf (get custom_messages::SumRequest :definition-)
      "float32 a
float32 b
---
float32 sum

")

(setf (get custom_messages::SumResponse :md5sum-) "210a18f816d5b88becb133b82fae0c4c")
(setf (get custom_messages::SumResponse :datatype-) "custom_messages/SumResponse")
(setf (get custom_messages::SumResponse :definition-)
      "float32 a
float32 b
---
float32 sum

")



(provide :custom_messages/Sum "210a18f816d5b88becb133b82fae0c4c")


