#ifndef ______________SmartObject
#define ______________SmartObject

#include <string>
#include <vector>
#include <map>

namespace NsAppLink
{
    namespace NsSmartObjects
    {
        class SmartObject;

        enum SmartType
        {
            SmartType_Null      = 0,
            SmartType_Boolean   = 1,
            SmartType_Integer   = 2,
            SmartType_Character = 3,
            SmartType_String    = 4,
            SmartType_Double    = 5,
            SmartType_Map       = 6,
            SmartType_Array     = 7,
            SmartType_Unknown   = -1
        };




        static const char* true_str = "true";
        static const char* false_str = "false";

        typedef std::vector<SmartObject> SmartArray;
        typedef std::map<std::string, SmartObject> SmartMap;

        class SmartObject
        {
        public:
            SmartObject();
            SmartObject(int);
            SmartObject(long);
            SmartObject(double);
            SmartObject(bool);
            SmartObject(char);
            SmartObject(const std::string);
            SmartObject(char*);
            SmartObject(const SmartObject&);
            ~SmartObject();

            operator int(void) const;
            operator double(void) const;
            operator char(void) const;
            operator bool(void) const;
            operator long(void) const;
            operator std::string(void) const;
            operator std::string*(void) const;
            const char* c_str(void) const;

            SmartObject& operator[](int);
            SmartObject& operator[](std::string);
            SmartObject& operator[](char*);
            SmartObject& operator[](const char*);

            SmartObject& operator=(long);
            SmartObject& operator=(int);
            SmartObject& operator=(double);
            SmartObject& operator=(bool);
            SmartObject& operator=(char);
            SmartObject& operator=(const std::string&);
            SmartObject& operator=(const char*);
            SmartObject& operator=(const SmartObject&);

            bool operator==(const SmartObject&) const;
            bool operator==(int) const;
            bool operator==(long) const;
            bool operator==(char) const;
            bool operator==(double) const;
            bool operator==(std::string) const;
            bool operator==(std::wstring) const;
            bool operator==(bool) const;

            SmartType get_type();

        protected:
            SmartType m_type;

            union {
                double double_value;
                bool bool_value;
                char char_value;
                long long_value;
                std::string* str_value;
                SmartArray* array_value;
                SmartMap* map_value;
            } m_data;

            inline void set_value_integer(long);
            inline void set_value_char(char);
            inline void set_value_double(double);
            inline void set_value_bool(bool);
            inline void set_value_string(const std::string&);
            inline void set_value_cstr(const char*);

            inline int convert_int(void) const;
            inline char convert_char(void) const;
            inline double convert_double(void) const;
            inline bool convert_bool(void) const;
            inline long convert_long(void) const;
            inline std::string convert_string(void) const;

            inline SmartObject& handle_map_access(std::string);
            inline SmartObject& handle_array_access(int index);

            static double convert_string_to_double(const std::string* s);
            static long convert_string_to_long(const std::string* s);

            void duplicate(const SmartObject&);

            void cleanup_data();
            void cleanup_data_if_type_changed_and_set_new_type(SmartType newType);

        };

        static const bool        invalid_bool_value   = false;
        static const int         invalid_int_value    = -1;
        static const char        invalid_char_value   = 0;
        static const std::string invalid_string_value = "";
        static const double      invalid_real_value   = -1;
        static const char*       invalid_cstr_value   = "";
        static SmartObject       invalid_object_value;

    }
}
#endif //#define ______________SmartObject