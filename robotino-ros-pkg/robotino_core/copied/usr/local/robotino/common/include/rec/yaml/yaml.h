#ifndef _REC_YAML_H_
#define _REC_YAML_H_

#include <QtCore>

namespace rec
{
	namespace yaml
	{
		typedef enum
		{
			StringEntry,
			ListEntry
		} entryType;

		class Reader
		{
		public:
			Reader();

			bool parse(const QString& fileName);

			QStringList keys() const;

			entryType entryTypeForKey(const QString& key) const;

			QString entryForKey(const QString& key) const;
			qreal realEntryForKey(const QString& key) const;

			QStringList listEntryForKey(const QString& key) const;
			QVector<qreal> realVectorEntryForKey(const QString& key) const;

		private:
			typedef QMap< QString, QStringList > entries_t;
			entries_t _entries;
		};
	}
}

#endif //_REC_YAML_H_
