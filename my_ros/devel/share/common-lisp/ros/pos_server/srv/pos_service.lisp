; Auto-generated. Do not edit!


(cl:in-package pos_server-srv)


;//! \htmlinclude pos_service-request.msg.html

(cl:defclass <pos_service-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass pos_service-request (<pos_service-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pos_service-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pos_service-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pos_server-srv:<pos_service-request> is deprecated: use pos_server-srv:pos_service-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pos_service-request>) ostream)
  "Serializes a message object of type '<pos_service-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pos_service-request>) istream)
  "Deserializes a message object of type '<pos_service-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pos_service-request>)))
  "Returns string type for a service object of type '<pos_service-request>"
  "pos_server/pos_serviceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pos_service-request)))
  "Returns string type for a service object of type 'pos_service-request"
  "pos_server/pos_serviceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pos_service-request>)))
  "Returns md5sum for a message object of type '<pos_service-request>"
  "ff8d7d66dd3e4b731ef14a45d38888b6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pos_service-request)))
  "Returns md5sum for a message object of type 'pos_service-request"
  "ff8d7d66dd3e4b731ef14a45d38888b6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pos_service-request>)))
  "Returns full string definition for message of type '<pos_service-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pos_service-request)))
  "Returns full string definition for message of type 'pos_service-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pos_service-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pos_service-request>))
  "Converts a ROS message object to a list"
  (cl:list 'pos_service-request
))
;//! \htmlinclude pos_service-response.msg.html

(cl:defclass <pos_service-response> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0))
)

(cl:defclass pos_service-response (<pos_service-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pos_service-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pos_service-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pos_server-srv:<pos_service-response> is deprecated: use pos_server-srv:pos_service-response instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <pos_service-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pos_server-srv:x-val is deprecated.  Use pos_server-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <pos_service-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pos_server-srv:y-val is deprecated.  Use pos_server-srv:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pos_service-response>) ostream)
  "Serializes a message object of type '<pos_service-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pos_service-response>) istream)
  "Deserializes a message object of type '<pos_service-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pos_service-response>)))
  "Returns string type for a service object of type '<pos_service-response>"
  "pos_server/pos_serviceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pos_service-response)))
  "Returns string type for a service object of type 'pos_service-response"
  "pos_server/pos_serviceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pos_service-response>)))
  "Returns md5sum for a message object of type '<pos_service-response>"
  "ff8d7d66dd3e4b731ef14a45d38888b6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pos_service-response)))
  "Returns md5sum for a message object of type 'pos_service-response"
  "ff8d7d66dd3e4b731ef14a45d38888b6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pos_service-response>)))
  "Returns full string definition for message of type '<pos_service-response>"
  (cl:format cl:nil "float32 x~%float32 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pos_service-response)))
  "Returns full string definition for message of type 'pos_service-response"
  (cl:format cl:nil "float32 x~%float32 y~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pos_service-response>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pos_service-response>))
  "Converts a ROS message object to a list"
  (cl:list 'pos_service-response
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'pos_service)))
  'pos_service-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'pos_service)))
  'pos_service-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pos_service)))
  "Returns string type for a service object of type '<pos_service>"
  "pos_server/pos_service")