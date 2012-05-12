/*!
  Sss - a slope soaring simulater.
  Copyright (C) 2002 Danny Chapman - flight@rowlhouse.freeserve.co.uk

  \file config_file.cpp
*/

#include "config_file.hh"

#include <fstream>
#include <cstdlib>
using namespace std;

const string Config_file::comment_char("#");
const string Config_file::delims(" \t\r");
const string Config_file::digits("1234567890.-");

void Config_file::reset(ifstream * & stream)
{
  TRACE_METHOD_ONLY(2);
  delete stream; 
  stream = new ifstream(m_file_name.c_str());
}

Config_file::Config_file(const string & file_name, bool & success)
  :
  m_file_name(file_name),
  m_seq_file_stream(new ifstream(m_file_name.c_str())),
  m_rand_file_stream(new ifstream(m_file_name.c_str()))
{
  TRACE_FILE_IF(1) {
    TRACE_METHOD(); TRACE("Config file %s\n", m_file_name.c_str());}

  success = true;
  /// \todo I need to look up how to best check the opening of the 
  /// file.
  char c;
  streampos pos_seq = m_seq_file_stream->tellg();
  streampos pos_rand = m_rand_file_stream->tellg();

  if (!m_seq_file_stream->get(c))
  {
    TRACE("Can't open %s\n", m_file_name.c_str());
    success = false;
  }
  if (!m_rand_file_stream->get(c))
  {
    TRACE("Can't open %s\n", m_file_name.c_str());
    success = false;
  }
  m_seq_file_stream->seekg(pos_seq);
  m_rand_file_stream->seekg(pos_rand);
}

/*!  Copy constructor - ensures that the new streams have the same
  offsets as the original, but they are new streams */
Config_file::Config_file(const Config_file & orig)
  :
  m_file_name(orig.m_file_name),
  m_seq_file_stream(new ifstream(m_file_name.c_str())),
  m_rand_file_stream(new ifstream(m_file_name.c_str()))
{
  TRACE_FILE_IF(1) {
    TRACE_METHOD(); TRACE("Copy of file %s\n", m_file_name.c_str());}
  // ensure that the streams are in the same position as the originals
  streampos pos = orig.m_seq_file_stream->tellg();
  m_seq_file_stream->seekg(pos);
  pos = orig.m_rand_file_stream->tellg();
  m_rand_file_stream->seekg(pos);
}

/*! ensures that the new streams have the same
  offsets as the original. */
Config_file Config_file::operator=(const Config_file & orig)
{
  TRACE_METHOD_ONLY(1);
  return Config_file(orig);
}

Config_file::~Config_file()
{
  TRACE_FILE_IF(1) {
    TRACE_METHOD(); TRACE("Destructor for config file %s\n", m_file_name.c_str());}

  if (m_seq_file_stream)
    delete m_seq_file_stream;
  if (m_rand_file_stream)
    delete m_rand_file_stream;
}

bool Config_file::get_next_processed_line(ifstream * stream,
                                          vector<string> & words)
{
 TRACE_METHOD_ONLY(3);
 if (!stream)
  {
    TRACE("input file is not open: %s\n", m_file_name.c_str());
    return false;
  }
  
  char char_line[1024];
  while (stream->getline(char_line, 255))
  {
    string::size_type beg_index, end_index;
    string line(char_line);
    words.clear();
    
//      cout << "=============================================================\n";
//      cout << line << endl;

    // remove comment
    beg_index = line.find(comment_char);
    if (beg_index !=  string::npos)
    {
      line.resize(beg_index);
//      cout << "After removing comment: " << line << endl;
    }

    beg_index = line.find_first_not_of(delims);

    while (beg_index != string::npos)
    {
      end_index = line.find_first_of(delims, beg_index);
      
      if (end_index == string::npos)
      {
        // end of word is end of line
        end_index = line.length();
      }
      
//      cout << "[" << beg_index << "," << end_index << "]";
      string a_word(line.substr(beg_index, end_index-beg_index));
//      cout << a_word.length() << " @" << a_word << "@" << endl;
      
      // now we have a word

      // convert true/false into 1/0
      if ( (a_word == "true") || 
           (a_word == "True") || 
           (a_word == "TRUE") )
      {
        a_word = "1";
      }
      else if ( (a_word == "false") || 
                (a_word == "False") || 
                (a_word == "FALSE") )
      {
        a_word = "0";
      }
          

      words.push_back(a_word);
      
      // find the next word
      beg_index = line.find_first_not_of(delims, end_index);
    }
    
    if (words.size() > 0)
    {
//        cout << "Individual words: @";
//        // output the words
//        for (unsigned int i = 0 ; i < words.size() ; ++i)
//        {
//          cout << words[i] << "@";
//        }
//        cout  << endl;
      
      // found our words - now go home!
      return true;
    }
    
    // words is empty - try again
  }
  
  // no more lines
  return false;
}


Config_file::Config_attr_value Config_file::get_next_attr_value()
{
  TRACE_METHOD_ONLY(3);
  Config_attr_value attr_value;
  
  while (true)
  {
    vector<string> words;
    if (false == get_next_processed_line(m_seq_file_stream, words))
    {
      // reached end of file
      return attr_value;
    }
    // we need at least one attribute and one value to do anything
    if (words.size() > 1)
    {
      // the easy bit
      attr_value.attr = words[0];
      
      // work out the type. We assume that all the values have the
      // same type, so just check the first.
      if ( (words[1] == "true") || (words[1] == "TRUE") )
      {
        attr_value.value_type = Config_attr_value::BOOL;
      }
      else if ( (words[1] == "false") || (words[1] == "FALSE") )
      {
        attr_value.value_type = Config_attr_value::BOOL;
      }
      else if (words[1].find_first_not_of(digits) != string::npos)
      {
        attr_value.value_type = Config_attr_value::STRING;
      }
      else
      {
        attr_value.value_type = Config_attr_value::FLOAT;
      }
      
      unsigned int i;
      for (i = 1 ; i < words.size() ; ++i)
      {
        Config_attr_value::Value val;
        switch ( attr_value.value_type )
        {
        case Config_attr_value::BOOL:
          val.bool_val = ( (words[i] == "true") || (words[i] == "TRUE") );
          break;

        case Config_attr_value::FLOAT:
          val.float_val = (float) atof(words[i].c_str());
          break;

        case Config_attr_value::STRING:
          val.string_val = words[i];
          break;

        default:
          break;
        }

        ++attr_value.num;
        attr_value.values.push_back(val);
      }
      // can return
      return attr_value;
    }
    // words.size() == 0, so have another go
  }
}


Config_file::Config_attr_value Config_file::find_next_attr_value(
  const string & attr)
{
  TRACE_METHOD_ONLY(3);
  Config_attr_value attr_value;
  
  while (true)
  {
    attr_value = get_next_attr_value();
    
    // have we reached the end of file?
    if (attr_value.num == 0)
      return attr_value;
    
    if (attr_value.attr == attr)
    {
      return attr_value;
    }
    // no luck yet - try next line
  }
  return attr_value;
}

bool Config_file::find_new_block(const string & block_name)
{
  Config_attr_value attr_value;
  
  while (true)
  {
    attr_value = find_next_attr_value("begin");
    if (Config_attr_value::INVALID == attr_value.value_type)
    {
      TRACE_FILE_IF(3)
        TRACE("Config_file::find_new_block() End of config file reached\n");
      return false;
    }
    if (Config_attr_value::STRING == attr_value.value_type)
    {
      if (block_name == attr_value.values[0].string_val)
      {
      TRACE_FILE_IF(3)
        TRACE("Config_file::find_new_block() Found block\n");
        return true;
      }
    }
  }
  // keep compiler happy
  return false;
}
