/*
  Sss - a slope soaring simulater.
  Copyright (C) 2002 Danny Chapman - flight@rowlhouse.freeserve.co.uk
*/
#ifndef CONFIG_FILE_H
#define CONFIG_FILE_H

#include "log_trace.hh"

#include <string>
#include <sstream>
#include <fstream>
#include <vector>
using namespace std;

//! Utility class for opening and parsing text config files
/*!
  
  Note that there are two ways of parsing configuration files:
  <ol>
  <li> Specifying an attribute that you suspect exists in the config
  file, and obtaining the value(s) associated with it.
  <li> Asking for the next attribute/value pair in the file.
  </ol>
  It is possible to mix-and-match the two methods with this class.
  
  The comment character in the config file is assumed to be "#".
*/
class Config_file{
public:
  //! success indicate ability to open file_name
  Config_file(const string & file_name, bool & success);
  //! Copy constructor
  Config_file(const Config_file & orig);
  //! Equals operator
  Config_file operator=(const Config_file & orig);
  
  //! Destructor closes any open file
  ~Config_file();

  //! Indicates the config file name
  string get_file_name() const {return m_file_name;}
  
  //! Function for reading a single value.
/*!  This returns the value for the first occurence of attr in the
  file. It does not affect the location used in the get_next
  functions.
  
  If the get fails, the value (passed by reference) is unmodified. 
  
  \return true if successful. False if either the attribute isn't
  found, or it has no value associated with it. In the case that
  multiple values are found, only the first is used (an error message
  is generated). */
  template<class T>
  bool get_value(const string & attr, T & value)
    {
      vector<T> values;
      bool retval = get_values(attr, values);
      if (false == retval)
      {
        return false;
      }
      if (values.size() == 0)
      {
        TRACE_FUNCTION(); TRACE("No values!\n");
        return false;
      }
      if (values.size() > 1)
      {
        TRACE_FILE_IF(1) 
          {
            TRACE_FUNCTION(); TRACE("Multiple values for attribute %s\n", attr.c_str());
          }
      }
      value = values[0];
      return true;
    }
  
  
  //! Version that asserts on failure
//! same as get_value, except asserts in case of failure
  template<class T>
  void get_value_assert(const string & attr, T & value)
    {
      bool retval = get_value(attr, value);
      if (retval != true)
      {
        TRACE("Failed to find attribute %s\n", attr.c_str());
      }
    }
  
  
  //! Get multiple values for an attribute
/*!  This returns the values for the first occurence of attr in the
  file. It does not affect the location used in the get_next
  functions.
  
  If the get fails, the values (passed by reference) is unmodified. 
  
  \return true if successful. False if the attribute is not found, or
  if it has no values associated with it.*/
  template<class T>
  bool get_values(const string & attr, vector<T> & values)
    {
      // reset the random access stream
      reset(m_rand_file_stream);
      
      vector<string> words;
      
      while (true == get_next_processed_line(m_rand_file_stream, words))
      {
        if ( (words.size() > 0) && (words[0] == attr) )
        {
          // found it!
          values.clear();
          if (1 == words.size())
          {
            TRACE("Found attribute %s but no value\n", attr.c_str());
            return false;
          }
          // we don't actually know the type of value, so use its ability
          // to read in from a stream.
          unsigned int i;
          string everything;
          for (i = 1 ; i < words.size() ; ++i)
          {
            everything += words[i];
            everything += " ";
          }
          istringstream is(everything);
          
          T value;
//      cout << "Read: " << attr;
          for (i = 1 ; i < words.size() ; ++i)
          {
            is >> value;
            values.push_back(value);
//        cout << value << " ";
          }
//      cout << endl;
          return true;
        }
      }
      // didn't find what we were looking for
      return false;
    }
  
  //! Contains the details of an attr/value configuration entry. Needs
  //! to be able to cope with multiple values, of an unknown type.
  struct Config_attr_value
  {
    Config_attr_value() : attr("None"), value_type(INVALID), num(0) {};
    string attr; //!< The attribute name
    //! INVALID is used if nothing found
    enum Value_type {INVALID, BOOL, FLOAT, STRING} value_type;
    unsigned int num; //!< Number of values - 0 if not found
    struct Value
    {
      bool   bool_val;
      float  float_val;
      string string_val;
    };
    vector<Value> values;
  };
  //! Returns the next attr/value entry. In the case of EOF, sets the
  //! type = INVALID, and num = 0
  Config_attr_value get_next_attr_value();
  
  //! Returns the next line of values as a vector (i.e. as in
  //! Config_attr_value but with no attr).
  template<class T>
  bool get_next_values(vector<T> & values, int num = 0)
    {
      values.clear();
      vector<string> words;

      if (false == get_next_processed_line(m_seq_file_stream, words))
      {
        // reached end of file
        return false;
      }          
      T value;
      if (num > 0)
      {
        if ((int) words.size() < num)
        {
          TRACE("expected %d values, found %d\n", num, words.size());
          TRACE("values read in were:\n");
          unsigned i;
          for (i = 0 ; i < words.size() ; ++i)
          {
            TRACE("%s\n", words[i].c_str());
          }
        }
        else if ((int) words.size() > num)
        {
          TRACE("expected %d values, found %d so ignoring the extras\n", words.size(), num);
          words.resize(num);
        }
      }
      for (unsigned i = 0 ; i < words.size() ; ++i)
      {
        istringstream is(words[i]);
        is >> value;
        values.push_back(value);
      }
      return true;
    }
  

  //! Gets the next value, asserting that it is of the specified
  //! attribute
  template<class T>
  void get_next_value_assert(const string & attr, T & value)
    {
      vector<string> words;
      
      while (true == get_next_processed_line(m_seq_file_stream, words))
      {
        if (words.size() < 2)
        {
          TRACE("Expecting at least two words with attribute %s\n",
                attr.c_str());
        }
        if (words[0] != attr)
        {
          TRACE("Expected attribute %s : found %s\n", 
                attr.c_str(), words[0].c_str());
        }
        
        // we don't actually know the type of value, so use its ability
        // to read in from a stream.
        istringstream is(words[1]);
//    cout << "Attribute: " << attr;
        is >> value;
//    cout << value << endl;
        return;
      }
      TRACE("Attribute %s not found\n", attr.c_str());
    }
  
  
  //! Gets the next value, WITHOUT asserting that it is of the specified
  //! attribute. value is unchanged if the attr is not as requested,
  //! and the read position does not advance (except for white-space). 
  //! the return value indicates if the read was successful.
  template<class T>
  bool get_next_value(const string & attr, T & value)
    {
      vector<string> words;
      // store position
      
      // Apparantly streampos should really be std::ios::pos_type, but
      // maybe my compiler is a bit old-fashioned?
      streampos pos = m_seq_file_stream->tellg();
      
      while (true == get_next_processed_line(m_seq_file_stream, words))
      {
        if (words[0] == attr)
        {
          // we don't actually know the type of value, so use its ability
          // to read in from a stream.
          istringstream is(words[1]);
//      cout << "Attribute: " << attr;
          is >> value;
//      cout << value << endl;
          return true;
        }
        else
        {
          // we have to rewind
          m_seq_file_stream->seekg(pos);
          return false;
        }
      }
      TRACE("Attribute %s not found\n", attr.c_str());
      return false;
    }
  

  /// finds the next block starting with "begin block_name". Returns
  /// true if found, false if not.
  bool find_new_block(const string & block_name);
  
  //! Finds the next occurence of the desired attribute, and returns
  //! the attr/value result
  Config_attr_value find_next_attr_value(const string & attr);
  
  /*!  resets the internal sequential file descriptor to point to the
    beginning of the file - so the next call to get_next_attr_value
    will return the first.  */
  void reset() {reset(m_seq_file_stream);}
  
  //! Returns the stream pointer used in get_next_attr_value(...).
  ifstream * get_stream() const {return m_seq_file_stream;}
  
private:
  //! Seeks the specified stream to the beginning of file
  void reset(ifstream * & stream);
  
  //! gets a vector of words for the next line. Returns false at EOF
  bool get_next_processed_line(ifstream * stream, vector<string> & words);
  
  const string m_file_name;
  ifstream * m_seq_file_stream;  //!< stream used for sequential access
  ifstream * m_rand_file_stream; //!< stream used for random access
  
  // some static consts
  static const string comment_char; //!< Comment character
  static const string delims;       //!< "word" delimiters
  static const string digits;       //!< used to identify numbers
  
};




#endif  


