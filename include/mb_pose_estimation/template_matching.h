#ifndef TEMPLATE_MATCHING_H
#define TEMPLATE_MATCHING_H

#include <string>

class TemplateMatching
{
public:
  typedef enum {SURF, SIFT} descriptor_t;

  TemplateMatching(descriptor_t descriptor_type = SIFT) : descriptor_type_(descriptor_type)
  {}

  virtual bool findTemplate(const std::string &template_image, const std::string &current_image);

  ~TemplateMatching()
  {}

private:
  descriptor_t descriptor_type_;
};

#endif
