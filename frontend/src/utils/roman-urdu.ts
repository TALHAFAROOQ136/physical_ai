/**
 * Roman Urdu transliteration utility
 * Converts Arabic-script Urdu to Roman Urdu (Urdu in Latin characters)
 */

// Mapping of Urdu characters to Roman equivalents
const URDU_TO_ROMAN_MAP: { [key: string]: string } = {
  // Vowels
  'ا': 'a',
  'آ': 'aa',
  'أ': 'a',
  'إ': 'i',
  'ئ': 'e',
  'ؤ': 'o',
  'و': 'o',
  'ُ': 'u',
  'َ': 'a',
  'ِ': 'i',
  'ے': 'ay',
  'ی': 'i',
  'ۃ': 't',
  'ء': '',
  'ۓ': 'y',

  // Consonants
  'ب': 'b',
  'ت': 't',
  'ٹ': 't',
  'ث': 's',
  'ج': 'j',
  'چ': 'ch',
  'ح': 'h',
  'خ': 'kh',
  'د': 'd',
  'ڈ': 'd',
  'ذ': 'z',
  'ر': 'r',
  'ڑ': 'r',
  'ز': 'z',
  'ژ': 'zh',
  'س': 's',
  'ش': 'sh',
  'ص': 's',
  'ض': 'z',
  'ط': 't',
  'ظ': 'z',
  'ع': 'a',
  'غ': 'gh',
  'ف': 'f',
  'ق': 'q',
  'ک': 'k',
  'گ': 'g',
  'ل': 'l',
  'م': 'm',
  'ن': 'n',
  'ں': 'n',
  'ه': 'h',
  'ھ': 'h',
  'و': 'w',
  'پ': 'p',
  'چ': 'ch',
  'ک': 'k',
  'گ': 'g',
  'ں': 'n',
  'ۂ': 'h',
  'ۃ': 't',
  'ۃ': 'tah',
};

// Additional common transliterations for words that don't follow simple character mapping
const WORD_TRANSLITERATIONS: { [key: string]: string } = {
  'کر': 'kar',
  'ہو': 'ho',
  'نہ': 'na',
  'ہی': 'hi',
  'اور': 'aur',
  'کہ': 'keh',
  'کے': 'ke',
  'کا': 'ka',
  'کی': 'ki',
  'لیے': 'liye',
  'ہے': 'hai',
  'ہیں': 'hain',
  'ہوں': 'hon',
  'ہوئی': 'hui',
  'ہوا': 'hua',
  'ہوئے': 'hue',
  'ہوتا': 'hota',
  'ہوتی': 'hoti',
  'ہوتے': 'hote',
  'کیا': 'kya',
  'کیوں': 'kyun',
  'کون': 'kon',
  'کہاں': 'kahan',
  'کب': 'kab',
  'کس': 'kis',
  'کسی': 'kisi',
  'کچھ': 'kuch',
  'تم': 'tum',
  'آپ': 'aap',
  'ہم': 'hum',
  'وہ': 'woh',
  'یہ': 'yeh',
  'ہمارا': 'hamara',
  'تمہارا': 'tumhara',
  'ان': 'in',
  'انہوں': 'inhon',
  'یہی': 'yehi',
  'ویسے': 'vaise',
  'بس': 'bas',
  'تو': 'to',
  'بھی': 'bhi',
  'صرف': 'sirf',
  'ہر': 'har',
  'ایک': 'ek',
  'دو': 'do',
  'تین': 'teen',
  'چار': 'char',
  'پانچ': 'panch',
  'چھ': 'chhay',
  'سات': 'sat',
  'آٹھ': 'aath',
  'نو': 'nau',
  'دس': 'das',
  'سال': 'saal',
  'ماہ': 'mah',
  'روز': 'roz',
  'دن': 'din',
  'رات': 'raat',
  'صبح': 'subah',
  'شام': 'shaam',
  'کام': 'kaam',
  'کتاب': 'kitab',
  'لڑکا': 'ladka',
  'لڑکی': 'ladki',
  'گھر': 'ghar',
  'سکول': 'school',
  'کمرہ': 'kamra',
  'پانی': 'paani',
  'ناشتا': 'nasta',
  'کھانا': 'khana',
  'چائے': 'chai',
  'دودھ': 'doodh',
  'چلو': 'chalo',
  'رکو': 'ruk jao',
  'اچھا': 'acha',
  'برا': 'bra',
  'بہتر': 'behtar',
  'بہت': 'bohat',
  'تھوڑا': 'thora',
  'زیادہ': 'ziada',
  'کم': 'kam',
  'جی': 'ji',
  'کرنا': 'karna',
  'ہونا': 'hona',
  'رہنا': 'rehna',
  'جانا': 'jana',
  'دینا': 'dena',
  'لینا': 'lena',
  'دیکھنا': 'dekhna',
  'سنا': 'suna',
  'لکھنا': 'likhna',
  'پڑھنا': 'padhna',
  'چلانا': 'chalana',
  'چلن': 'chalan',
  'روٹی': 'roti',
  'کپڑے': 'kapray',
  'مہنگا': 'mehnga',
  'سستا': 'sasta',
  'پرانا': 'purana',
  'نیا': 'naya',
  'پرانی': 'purani',
  'نئی': 'nayi',
  'بچے': 'bachay',
  'والدین': 'waldeen',
  'چچا': 'chacha',
  'خالہ': 'khala',
  'کزن': 'cousin',
  'دوست': 'dost',
  'اچھے': 'ache',
  'بڑا': 'bara',
  'چھوٹا': 'chota',
  'بڑی': 'bari',
  'چھوٹی': 'choti',
  'کامیاب': 'kamyab',
  'ناکام': 'nakam',
  'ناقص': 'naqis',
  'نفع': 'nafa',
  'نقصان': 'nuqsan',
  'نصیحت': 'naseehat',
  'نصاب': 'nizam',
  'نظر': 'nazar',
  'نظام': 'nizam',
  'نیا': 'naya',
  'نۓ': 'naye',
  'نئے': 'naye',
  'سائنس': 'science',
  'ٹیکنالوجی': 'technology',
  'روبوٹکس': 'robotics',
  'آرٹیفیشل': 'artificial',
  'انٹیلی جنس': 'intelligence',
  'فزیکل': 'physical',
  'ای': 'ai',
  'ای آئی': 'ai',
  'ہیومنوائڈ': 'humanoid',
  'روبوٹ': 'robot',
  'پروگرامنگ': 'programming',
  'کوڈ': 'code',
  'کمپیوٹر': 'computer',
  'سسٹم': 'system',
  'ڈیٹا': 'data',
  'ایل ایم': 'llm',
  'ایل ایم': 'large language model',
  'راگ': 'rag',
  'ریٹریول': 'retrieval',
  'ایوی ایشن': 'aviation',
  'میکانکس': 'mechanics',
  'ایکٹو ایٹرز': 'actuators',
  'سینسرز': 'sensors',
  'کنٹرول': 'control',
  'الگوردم': 'algorithm',
  'میشین': 'machine',
  'لرننگ': 'learning',
  'وژن': 'vision',
  'لینگویج': 'language',
  'ایکشن': 'action',
  'وی ایل ای': 'vla',
  'ایس ایس': 'iss',
  'ار اوس': 'ros',
  'ار او ایس': 'ros',
  'نیویڈیا': 'nvidia',
  'آئسک': 'isaac',
  'سیمولیشن': 'simulation',
  'گزیبو': 'gazebo',
  'یو ار ڈی ایف': 'urdf',
  'نوڈز': 'nodes',
  'ٹوپکس': 'topics',
  'سروسز': 'services',
  'ایکشنز': 'actions',
  'فریم ورک': 'framework',
  'لیبیریری': 'library',
  'ایپلی کیشن': 'application',
  'انویئیشن': 'innovation',
  'ایجاد': 'ijad',
  'تجربہ': 'tajurba',
  'کھوج': 'khoj',
  'تحقیق': 'tahqiq',
  'نئے': 'naye',
};

/**
 * Transliterates Urdu text to Roman Urdu
 * @param text - Urdu text in Arabic script
 * @returns Roman Urdu text in Latin characters
 */
export function transliterateToRomanUrdu(text: string): string {
  if (!text || typeof text !== 'string') {
    return '';
  }

  // Replace common words first
  let result = text;
  Object.entries(WORD_TRANSLITERATIONS).forEach(([urduWord, romanWord]) => {
    // Create a regex that finds the word surrounded by word boundaries or punctuation
    const regex = new RegExp(`\\b${urduWord}\\b`, 'g');
    result = result.replace(regex, romanWord);
  });

  // Then transliterate remaining characters
  let transliterated = '';
  for (let i = 0; i < result.length; i++) {
    const char = result[i];
    if (URDU_TO_ROMAN_MAP[char]) {
      transliterated += URDU_TO_ROMAN_MAP[char];
    } else {
      // If not in map, keep the character (spaces, punctuation, numbers remain unchanged)
      transliterated += char;
    }
  }

  return transliterated;
}

/**
 * Adds Roman Urdu support as a new locale option
 */
export const ROMAN_URDU_LOCALE = 'ur-r';

/**
 * Determines if a text is likely Arabic-script Urdu
 * @param text - Text to analyze
 * @returns Boolean indicating if text is likely Urdu
 */
export function isArabicScriptUrdu(text: string): boolean {
  if (!text) return false;

  // Check if text contains Arabic/Persian characters (Unicode ranges)
  const arabicRegex = /[\u0600-\u06FF\u0750-\u077F]/;
  return arabicRegex.test(text);
}

/**
 * Checks if text is already Roman Urdu
 * @param text - Text to analyze
 * @returns Boolean indicating if text is Roman Urdu
 */
export function isRomanUrdu(text: string): boolean {
  if (!text) return false;

  // Roman Urdu is typically in Latin characters but may contain some Arabic script
  // A simple heuristic: if mostly Latin characters, it's probably Roman Urdu
  const latinChars = text.match(/[a-zA-Z]/g) || [];
  const totalChars = text.replace(/\s/g, '').length;

  return totalChars > 0 && (latinChars.length / totalChars) > 0.6;
}

/**
 * Smart transliteration function that detects input script and converts appropriately
 * @param text - Input text (Arabic-script Urdu or Roman Urdu)
 * @returns Converted text (Roman Urdu)
 */
export function smartTransliterateToRomanUrdu(text: string): string {
  if (!text) return '';

  if (isRomanUrdu(text)) {
    // Already Roman Urdu, return as-is
    return text;
  } else if (isArabicScriptUrdu(text)) {
    // Convert from Arabic-script to Roman Urdu
    return transliterateToRomanUrdu(text);
  }

  // If neither, return as-is
  return text;
}