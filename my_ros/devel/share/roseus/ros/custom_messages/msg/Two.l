;; Auto-generated. Do not edit!


(when (boundp 'custom_messages::Two)
  (if (not (find-package "CUSTOM_MESSAGES"))
    (make-package "CUSTOM_MESSAGES"))
  (shadow 'Two (find-package "CUSTOM_MESSAGES")))
(unless (find-package "CUSTOM_MESSAGES::TWO")
  (make-package "CUSTOM_MESSAGES::TWO"))

(in-package "ROS")
;;//! \htmlinclude Two.msg.html


(defclass custom_messages::Two
  :super ros::object
  :slots (_a _b ))

(defmethod custom_messages::Two
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

(setf (get custom_messages::Two :md5sum-) "3f6326d24b4937f854196fd3a843f42e")
(setf (get custom_messages::Two :datatype-) "custom_messages/Two")
(setf (get custom_messages::Two :definition-)
      "float32 a
float32 b

")



(provide :custom_messages/Two "3f6326d24b4937f854196fd3a843f42e")


