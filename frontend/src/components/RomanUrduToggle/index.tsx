import React, { useState, useEffect } from 'react';
import { transliterateToRomanUrdu, isArabicScriptUrdu, smartTransliterateToRomanUrdu } from '../../utils/roman-urdu';

interface RomanUrduToggleProps {
  children: string;
  className?: string;
}

const RomanUrduToggle: React.FC<RomanUrduToggleProps> = ({ children, className = '' }) => {
  const [showRomanUrdu, setShowRomanUrdu] = useState(false);
  const [convertedText, setConvertedText] = useState('');

  useEffect(() => {
    // Check if the text is Arabic-script Urdu and convert it if needed
    if (isArabicScriptUrdu(children)) {
      const romanText = smartTransliterateToRomanUrdu(children);
      setConvertedText(romanText);
    } else {
      setConvertedText(children);
    }
  }, [children]);

  // If it's not Arabic-script Urdu, just return the original text
  if (!isArabicScriptUrdu(children)) {
    return <span className={className}>{children}</span>;
  }

  return (
    <div className={`roman-urdu-container ${className}`}>
      <div className="urdu-display">
        {showRomanUrdu ? (
          <span className="roman-urdu-text">{convertedText}</span>
        ) : (
          <span className="arabic-urdu-text">{children}</span>
        )}
      </div>
      <button
        onClick={() => setShowRomanUrdu(!showRomanUrdu)}
        className="roman-urdu-toggle-btn"
        aria-label={showRomanUrdu ? "Show Arabic Urdu" : "Show Roman Urdu"}
      >
        {showRomanUrdu ? "اُردو" : "Roman Urdu"}
      </button>
    </div>
  );
};

export default RomanUrduToggle;