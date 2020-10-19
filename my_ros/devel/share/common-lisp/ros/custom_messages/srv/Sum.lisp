; Auto-generated. Do not edit!


(cl:in-package custom_messages-srv)


;//! \htmlinclude Sum-request.msg.html

(cl:defclass <Sum-request> (roslisp-msg-protocol:ros-message)
  ((a
    :reader a
    :initarg :a
    :type cl:float
    :initform 0.0)
   (b
    :reader b
    :initarg :b
    :type cl:float
    :initform 0.0))
)

(cl:defclass Sum-request (<Sum-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Sum-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Sum-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_messages-srv:<Sum-request> is deprecated: use custom_messages-srv:Sum-request instead.")))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <Sum-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_messages-srv:a-val is deprecated.  Use custom_messages-srv:a instead.")
  (a m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <Sum-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_messages-srv:b-val is deprecated.  Use custom_messages-srv:b instead.")
  (b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Sum-request>) ostream)
  "Serializes a message object of type '<Sum-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'b))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Sum-request>) istream)
  "Deserializes a message object of type '<Sum-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'b) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Sum-request>)))
  "Returns string type for a service object of type '<Sum-request>"
  "custom_messages/SumRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Sum-request)))
  "Returns string type for a service object of type 'Sum-request"
  "custom_messages/SumRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Sum-request>)))
  "Returns md5sum for a message object of type '<Sum-request>"
  "210a18f816d5b88becb133b82fae0c4c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Sum-request)))
  "Returns md5sum for a message object of type 'Sum-request"
  "210a18f816d5b88becb133b82fae0c4c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Sum-request>)))
  "Returns full string definition for message of type '<Sum-request>"
  (cl:format cl:nil "float32 a~%float32 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Sum-request)))
  "Returns full string definition for message of type 'Sum-request"
  (cl:format cl:nil "float32 a~%float32 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Sum-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Sum-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Sum-request
    (cl:cons ':a (a msg))
    (cl:cons ':b (b msg))
))
;//! \htmlinclude Sum-response.msg.html

(cl:defclass <Sum-response> (roslisp-msg-protocol:ros-message)
  ((sum
    :reader sum
    :initarg :sum
    :type cl:float
    :initform 0.0))
)

(cl:defclass Sum-response (<Sum-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Sum-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Sum-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_messages-srv:<Sum-response> is deprecated: use custom_messages-srv:Sum-response instead.")))

(cl:ensure-generic-function 'sum-val :lambda-list '(m))
(cl:defmethod sum-val ((m <Sum-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_messages-srv:sum-val is deprecated.  Use custom_messages-srv:sum instead.")
  (sum m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Sum-response>) ostream)
  "Serializes a message object of type '<Sum-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'sum))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Sum-response>) istream)
  "Deserializes a message object of type '<Sum-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'sum) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Sum-response>)))
  "Returns string type for a service object of type '<Sum-response>"
  "custom_messages/SumResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Sum-response)))
  "Returns string type for a service object of type 'Sum-response"
  "custom_messages/SumResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Sum-response>)))
  "Returns md5sum for a message object of type '<Sum-response>"
  "210a18f816d5b88becb133b82fae0c4c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Sum-response)))
  "Returns md5sum for a message object of type 'Sum-response"
  "210a18f816d5b88becb133b82fae0c4c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Sum-response>)))
  "Returns full string definition for message of type '<Sum-response>"
  (cl:format cl:nil "float32 sum~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Sum-response)))
  "Returns full string definition for message of type 'Sum-response"
  (cl:format cl:nil "float32 sum~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Sum-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Sum-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Sum-response
    (cl:cons ':sum (sum msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Sum)))
  'Sum-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Sum)))
  'Sum-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Sum)))
  "Returns string type for a service object of type '<Sum>"
  "custom_messages/Sum")